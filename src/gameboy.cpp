#include "gameboy.h"

void GameBoy::reset()
{
    cpu = CPU{};
    ppu = PPU{};
    apu = APU{};
    apu.sample_buffer.reserve(2048); // ~3 frames of stereo samples at 44100Hz
    memory = Memory{};

    memory.apu = &apu; // Link APU to memory for audio register access
    memory.gameboy = this; // Link GameBoy to memory for cycle callbacks

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
}

void GameBoy::load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path)
{
    reset();
    memory.load_rom(rom);
    memory.load_battery(rom_path); // Load battery RAM if applicable
}

int GameBoy::step()
{
    return cpu.execute_instruction(memory);
}

int GameBoy::step_frame()
{
    int cycles_executed = 0;
    while (cycles_executed < CYCLES_PER_FRAME)
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
                ppu.step(cycles, memory);
                apu.step(cycles, memory);
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
        cycles_executed += cycles;
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
    cpu.sp -= 2;
    cpu.bus_write_word(memory, cpu.sp, cpu.pc);
    
    // Jump to interrupt vector
    cpu.pc = interrupt_vector;
    cpu.tick_internal(memory);
}

void GameBoy::on_memory_cycle(int cycles)
{
    ppu.step(cycles, memory);
    apu.step(cycles, memory);
    memory.tick_timers(cycles);

}
