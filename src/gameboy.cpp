#include "gameboy.h"

void GameBoy::reset() {
    memory = Memory{};
    cpu = cpu::CPU{};
    ppu = PPU{};

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

void GameBoy::load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path) {
    reset();
    memory.load_rom(rom);
    memory.load_battery(rom_path); // Load battery RAM if applicable
}

int GameBoy::step() {
    return cpu.execute_instruction(memory);
}

int GameBoy::step_frame() {
    int cycles_executed = 0;
    while (cycles_executed < CYCLES_PER_FRAME) {
        // Apply scheduled IME enable (from previous EI) before checking interrupts
        if (cpu.ime_scheduled) {
            cpu.ime = true;
            cpu.ime_scheduled = false;
        }

        // Check for wake from HALT/STOP
        uint8_t if_reg = memory.read(IO_IF);
        uint8_t ie_reg = memory.read(IO_IE);
        uint8_t pending = if_reg & ie_reg & 0x1F;
        
        if(cpu.halted && pending) {
            cpu.halted = false; // Wake from HALT
        }

        if(cpu.stopped && pending) {
            cpu.stopped = false; // Wake from STOP on any interrupt
        }

        int cycles;
        if(cpu.halted || cpu.stopped) {
            cycles = 4; // HALT and STOP consume 4 cycles while halted/stopped
        } else {
            handle_interrupts();
            cycles = step();
        }

        if (cycles < 0) {
            running = false;
            return cycles; // Error occurred
        }
        cycles_executed += cycles;

        // Update PPU and timers
        if (!cpu.stopped) {
            ppu.step(cycles, memory);
        }
        memory.tick_timers(cycles);
    }
    return cycles_executed;
}

void GameBoy::handle_interrupts() {
    if (!cpu.ime) {
        return; // Interrupts are disabled
    }
    
    uint8_t if_reg = memory.read(IO_IF);
    uint8_t ie_reg = memory.read(IO_IE);
    uint8_t triggered = if_reg & ie_reg & 0x1F; // Check which interrupts are both flagged and enabled
    
    if (triggered == 0) {
        return; // No interrupts to handle
    }
    
    // Disable interrupts
    cpu.ime = false;
    
    // Determine which interrupt to service (priority order: VBlank, LCD STAT, Timer, Serial, Joypad)
    uint8_t interrupt_bit = 0;
    uint16_t interrupt_vector = 0;
    
    if (triggered & INT_VBLANK) {
        interrupt_bit = INT_VBLANK;
        interrupt_vector = INT_VECTOR_VBLANK;
    } else if (triggered & INT_LCD_STAT) {
        interrupt_bit = INT_LCD_STAT;
        interrupt_vector = INT_VECTOR_LCD_STAT;
    } else if (triggered & INT_TIMER) {
        interrupt_bit = INT_TIMER;
        interrupt_vector = INT_VECTOR_TIMER;
    } else if (triggered & INT_SERIAL) {
        interrupt_bit = INT_SERIAL;
        interrupt_vector = INT_VECTOR_SERIAL;
    } else if (triggered & INT_JOYPAD) {
        interrupt_bit = INT_JOYPAD;
        interrupt_vector = INT_VECTOR_JOYPAD;
    }
    
    // Clear the interrupt flag
    memory.write(IO_IF, if_reg & ~interrupt_bit);
    
    // Push PC onto stack
    cpu.sp -= 2;
    memory.write_word(cpu.sp, cpu.pc);
    
    // Jump to interrupt vector
    cpu.pc = interrupt_vector;
}


