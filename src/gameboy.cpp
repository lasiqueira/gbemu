#include "gameboy.h"
#include "compat_palettes.h"
#include <cstring>

void GameBoy::reset()
{
    cpu = CPU{};
    ppu = PPU{};
    apu = APU{};
    apu.sample_buffer.reserve(2048); // ~3 frames of stereo samples at 44100Hz
    memory = Memory{};
    
    //requested model must be persistent
    model = Model::DMG; // Default actual model
    cgb_mode = false;
    dmg_compat = false;

    memory.apu = &apu; // Link APU to memory for audio register access
    memory.gameboy = this; // Link GameBoy to memory for cycle callbacks
}

void GameBoy::load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path)
{
    reset();
    memory.load_rom(rom);

    // Determine if the cartridge is CGB capable
    set_cartridge_mode();

    apply_post_boot_state(); // Apply post-boot I/O register state

    memory.load_battery(rom_path); // Load battery RAM if applicable
}

int GameBoy::step()
{
    return cpu.execute_instruction(memory);
}

int GameBoy::step_frame()
{
    // A frame ends when the PPU enters V-Blank, so the buffer handed to the display is always one
    // complete frame. The cycle budget only matters while the LCD is off and V-Blank never comes.
    ppu.frame_ready = false;
    int cycles_executed = 0;
    while (!ppu.frame_ready && cycles_executed < CYCLES_PER_FRAME)
    {
        // Apply scheduled IME enable (from previous EI) before checking interrupts
        if (cpu.ime_scheduled) {
            cpu.ime = true;
            cpu.ime_scheduled = false;
        }

        // Check for wake from HALT/STOP
        if (cpu.halted || cpu.stopped)
        {
            uint8_t pending = memory.read_io_raw(IO_IF) & memory.ie_register & INT_ALL_MASK;
            if (cpu.halted && pending) cpu.halted = false;
            if (cpu.stopped && pending) cpu.stopped = false;
        }

        int cycles;
        if (cpu.halted || cpu.stopped)
        {
            cycles = 4; // HALT and STOP consume 4 cycles while halted/stopped
            // No CPU bus access happens while halted/stopped, so tick subsystems explicitly here
            if (!cpu.stopped)
            {
                int sys = memory.to_real_time(cycles);
                ppu.step(sys, memory);
                apu.step(sys, cycles, memory);
            }
            memory.tick_timers(cycles);
        }
        else
        {
            handle_interrupts();
            cycles = step(); // PPU/APU/timers are ticked inline via bus_read/bus_write as the instruction runs
        }

        if (cycles < 0)
        {
            running = false;
            return cycles; // Error occurred
        }

        if (memory.gdma_requested)
        {
            memory.gdma_requested = false;
            while (memory.hdma_blocks_left > 0)
            {
                cycles += hdma_copy_block();
            }
        }

        if (ppu.hblank_started)
        {
            ppu.hblank_started = false; // Reset the flag after handling HBlank
            if (memory.hdma_active && !cpu.halted)
            {
                cycles += hdma_copy_block();
                if (memory.hdma_blocks_left == 0)
                {
                    memory.hdma_active = false;
                }
            }
        }

        int real_cycles = memory.to_real_time(cycles); // real-time cycles so a frame stays 70224; taken before any speed switch
        if (cpu.stop_executed)
        {
            cpu.stop_executed = false;
            real_cycles += try_speed_switch();
        }
        cycles_executed += real_cycles;
    }
    return cycles_executed;
}

void GameBoy::handle_interrupts()
{
    if (!cpu.ime)
    {
        return; // Interrupts are disabled
    }
    
    uint8_t if_reg = memory.read_io_raw(IO_IF);
    uint8_t ie_reg = memory.ie_register;
    uint8_t triggered = if_reg & ie_reg & INT_ALL_MASK; // Check which interrupts are both flagged and enabled
    
    if (triggered == 0)
    {
        return; // No interrupts to handle
    }
    
    // Disable interrupts
    cpu.ime = false;
    
    // Determine which interrupt to service (priority order: VBlank, LCD STAT, Timer, Serial, Joypad)
    uint8_t interrupt_bit = 0;
    uint16_t interrupt_vector = 0;
    
    if (triggered & INT_VBLANK)
    {
        interrupt_bit = INT_VBLANK;
        interrupt_vector = INT_VECTOR_VBLANK;
    }
    else if (triggered & INT_LCD_STAT)
    {
        interrupt_bit = INT_LCD_STAT;
        interrupt_vector = INT_VECTOR_LCD_STAT;
    }
    else if (triggered & INT_TIMER)
    {
        interrupt_bit = INT_TIMER;
        interrupt_vector = INT_VECTOR_TIMER;
    }
    else if (triggered & INT_SERIAL)
    {
        interrupt_bit = INT_SERIAL;
        interrupt_vector = INT_VECTOR_SERIAL;
    }
    else if (triggered & INT_JOYPAD)
    {
        interrupt_bit = INT_JOYPAD;
        interrupt_vector = INT_VECTOR_JOYPAD;
    }
    
    // Clear the interrupt flag
    memory.write(IO_IF, if_reg & ~interrupt_bit);
    
    // Interrupt dispatch takes 5 M-cycles on hardware: 2 internal, 2 pushing PC, 1 jumping to the vector
    cpu.tick_internal(memory);
    cpu.tick_internal(memory);
    
    // Push PC onto stack
    if (is_oam_bug_range(cpu.sp))
    {
        memory.oam_bug(OamCorruption::Write); // glitched write from the implied DEC SP
    }
    cpu.sp -= 2;
    cpu.bus_write_word(memory, cpu.sp, cpu.pc);
    
    // Jump to interrupt vector
    cpu.pc = interrupt_vector;
    cpu.tick_internal(memory);
}

void GameBoy::on_memory_cycle(int cycles)
{
    int sys = memory.to_real_time(cycles);
    ppu.step(sys, memory);
    apu.step(sys, cycles, memory);
    memory.tick_timers(cycles); // DIV/TIMA: CPU clock,     don't halve
}

void GameBoy::set_cartridge_mode()
{
    bool cart_is_cgb = (memory.cgb_flag == CGB_FLAG_ENHANCED || memory.cgb_flag == CGB_FLAG_ONLY);
    model = model_is_auto ? (cart_is_cgb ? Model::CGB : Model::DMG) : requested_model;
    cgb_mode = (model == Model::CGB);
    ppu.cgb_mode = cgb_mode;
    apu.cgb_mode = cgb_mode;
    dmg_compat = (cgb_mode && !cart_is_cgb);
    ppu.dmg_compat = dmg_compat;
    memory.cgb_mode = cgb_mode;
    memory.dmg_compat = dmg_compat;
}

// Writes 4 RGB555 colours into one palette of a CRAM bank (little-endian, 8 bytes per palette).
static void write_cram_palette(uint8_t* cram, int palette, const uint16_t (&colours)[COLOURS_PER_PALETTE])
{
    for (int i = 0; i < COLOURS_PER_PALETTE; ++i)
    {
        cram[palette * CRAM_PALETTE_BYTES + i * CRAM_COLOUR_BYTES] = colours[i] & 0xFF;
        cram[palette * CRAM_PALETTE_BYTES + i * CRAM_COLOUR_BYTES + 1] = colours[i] >> 8;
    }
}

void GameBoy::apply_post_boot_state()
{
    // Post-boot I/O register state (skipping boot ROM)
    memory.write(IO_JOYPAD, 0x3F);
    memory.write(IO_LCDC, 0x91); // LCD on, BG on
    memory.write(IO_STAT, 0x00);
    memory.write(IO_SCY, 0x00);
    memory.write(IO_SCX, 0x00);
    memory.write(IO_LY, 0x00);
    memory.write(IO_LYC, 0x00);
    memory.write(IO_BGP, 0xFC);  // Background palette (11 11 10 00)
    memory.write(IO_OBP0, 0xFF);
    memory.write(IO_OBP1, 0xFF);
    memory.write(IO_WY, 0x00);
    memory.write(IO_WX, 0x00);
    memory.write(IO_IF, 0x00);

    if (cgb_mode)
    {
        cpu.af.pair = POST_BOOT_AF_CGB;
        cpu.bc.pair = POST_BOOT_BC_CGB;
        cpu.de.pair = POST_BOOT_DE_CGB;
        cpu.hl.pair = POST_BOOT_HL_CGB;

        memory.write_io_raw(IO_SVBK, SVBK_UNUSED_BITS); // WRAM bank 1
        memory.write_io_raw(IO_KEY1, KEY1_UNUSED_BITS); // Normal speed, switch not armed
        memory.write_io_raw(IO_OPRI, dmg_compat ? OPRI_DMG_PRIORITY : 0x00); // CGB (OAM-order) object priority
        for (size_t i = 0; i < sizeof(apu.wave_ram); ++i) // CGB boot ROM leaves wave RAM as 00 FF 00 FF ...
            apu.wave_ram[i] = (i & 1) ? 0xFF : 0x00;
        std::memset(memory.bg_palette_ram, 0xFF, sizeof(memory.bg_palette_ram));
        if (dmg_compat)
        {
            CompatPalettes p = pick_compat_palettes(memory.rom);
            write_cram_palette(memory.bg_palette_ram, 0, p.bg);
            write_cram_palette(memory.obj_palette_ram, 0, p.obj0);
            write_cram_palette(memory.obj_palette_ram, 1, p.obj1);

            cpu.bc.pair = static_cast<uint16_t>(p.checksum) << 8; // B = checksum, C = 0
            cpu.de.pair = POST_BOOT_DE_CGB_COMPAT;
            cpu.hl.pair = p.logo_tilemap ? POST_BOOT_HL_CGB_COMPAT_LOGO : POST_BOOT_HL_CGB_COMPAT;
        }
        ppu.rebuild_palette_cache(memory);
    }
}

int GameBoy::try_speed_switch()
{
    if (!cgb_mode || dmg_compat || !memory.key1_armed) return 0;
    memory.double_speed = !memory.double_speed;
    memory.key1_armed = false;
    memory.div_counter = 0;
    memory.write_io_raw(IO_DIV, 0);
    for (int i = 0; i < SPEED_SWITCH_STALL_CYCLES / 4; ++i)
    {
        ppu.step(4, memory);
        apu.step(4, 0, memory); // cpu_cycles = 0: DIV is frozen
    }

    return SPEED_SWITCH_STALL_CYCLES; // real-time cycles spent in the stall
}

int GameBoy::hdma_copy_block()
{
    for (int i = 0; i < HDMA_BLOCK_SIZE; ++i)
    {
        uint8_t value = memory.read(memory.hdma_source + i);
        memory.vram[memory.vram_bank * VRAM_BANK_SIZE + memory.hdma_dest + i] = value;
    }
    memory.hdma_source += HDMA_BLOCK_SIZE;
    memory.hdma_dest += HDMA_BLOCK_SIZE;
    memory.hdma_blocks_left--;

    if (memory.hdma_dest >= VRAM_BANK_SIZE) // destination ran past $9FFF: the transfer stops early
    {
        memory.hdma_dest &= (VRAM_BANK_SIZE - 1) & ~(HDMA_BLOCK_SIZE - 1);
        memory.hdma_blocks_left = 0;
    }

    int m_cycles = memory.double_speed ? 16 : 8; // ~8 us of real time in both speeds

    for (int i = 0; i < m_cycles; ++i)
    {
        on_memory_cycle(4);
    }
    return m_cycles * 4; // CPU cycles stalled, at the current speed
}
