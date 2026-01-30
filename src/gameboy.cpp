#include "gameboy.h"
#include <print>

GameBoy::GameBoy() : running(false) {
    memory.write(IO_LY, 0); // Initialize LY register
    memory.write(IO_IF, 0); // Initialize IF register (no interrupts pending)
    
    // Initialize joypad register (0xFF00) - bits 4-5 set (no keys selected), bits 0-3 set (no buttons pressed)
    memory.write(0xFF00, 0x3F);
    
    // Initialize LCD registers to post-boot state (skipping boot ROM)
    memory.write(0xFF40, 0x91); // LCDC: LCD on, BG on, BG tile data at 8800-97FF
    memory.write(0xFF41, 0x00); // STAT: Mode 0
    memory.write(0xFF42, 0x00); // SCY: Scroll Y = 0
    memory.write(0xFF43, 0x00); // SCX: Scroll X = 0
    memory.write(0xFF44, 0x00); // LY: Line = 0
    memory.write(0xFF45, 0x00); // LYC: LY Compare = 0
    memory.write(0xFF47, 0xFC); // BGP: Background palette (11 11 10 00)
    memory.write(0xFF48, 0xFF); // OBP0: Object palette 0
    memory.write(0xFF49, 0xFF); // OBP1: Object palette 1
    memory.write(0xFF4A, 0x00); // WY: Window Y = 0
    memory.write(0xFF4B, 0x00); // WX: Window X = 0
}

void GameBoy::load_rom(const std::vector<uint8_t>& rom) {
    memory.load_rom(rom);
}

int GameBoy::step() {
    return cpu.execute_instruction(memory);
}

int GameBoy::step_frame() {
    int cycles_executed = 0;
    while (cycles_executed < CYCLES_PER_FRAME) {
        // Handle delayed IME enable (must be done before interrupt check)
        if (cpu.ime_scheduled) {
            cpu.ime = true;
            cpu.ime_scheduled = false;
        }
        
        // Handle interrupts before executing the next instruction
        handle_interrupts();
        
        int cycles = step();
        if (cycles < 0) {
            running = false;
            return cycles; // Error occurred
        }
        cycles_executed += cycles;
        
        // Update PPU
        ppu.step(cycles, memory);
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


