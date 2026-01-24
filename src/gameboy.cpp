#include "gameboy.h"
#include <print>

GameBoy::GameBoy() : running(false), lcd_cycles(0), ly(0) {
    memory.write(IO_LY, ly); // Initialize LY register
    memory.write(IO_IF, 0); // Initialize IF register (no interrupts pending)
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
        // Handle interrupts before executing the next instruction
        handle_interrupts();
        
        int cycles = step();
        if (cycles < 0) {
            running = false;
            return cycles; // Error occurred
        }
        cycles_executed += cycles;
        
        // TODO: Update other components (PPU, timers, APU) with cycles
        update_lcd(cycles);
    }
    return cycles_executed;
}

void GameBoy::update_lcd(int cycles) {
    lcd_cycles += cycles;
    while (lcd_cycles >= CYCLES_PER_SCANLINE) {
        lcd_cycles -= CYCLES_PER_SCANLINE;
        ly++;
        if (ly >= SCANLINES_PER_FRAME) {
            ly = 0; // Reset to first scanline
        }
        
        // Trigger VBlank interrupt when entering VBlank period
        if (ly == VBLANK_SCANLINE) {
            uint8_t if_reg = memory.read(IO_IF);
            memory.write(IO_IF, if_reg | INT_VBLANK); // Set VBlank interrupt flag
        }
        
        memory.write(IO_LY, ly); // Update LY register in memory
        
#ifdef GBEMU_DEBUG
        // Debug: Print when LY reaches 145
        static int debug_counter = 0;
        if (ly == 145 && ++debug_counter <= 5) {
            std::println("DEBUG: LY reached 145 (count: {})", debug_counter);
        }
#endif
    }
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

void GameBoy::run() {
    running = true;
    int frames_executed = 0;
    
    while (running) {
        int cycles = step_frame();
        if (cycles < 0) {
            return; // Exit on error
        }
        frames_executed++;
        
        // Print progress every 60 frames (roughly 1 second)
        if (frames_executed % 60 == 0) {
            std::println("Frame {}...", frames_executed);
        }
        
        // TODO: Render frame, handle input, etc.
    }
    
    std::println("\nEmulation stopped after {} frames", frames_executed);
}
