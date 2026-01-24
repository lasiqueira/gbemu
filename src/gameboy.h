#pragma once
#include <cstdint>
#include <vector>
#include "cpu.h"
#include "memory.h"

// Game Boy timing constants
constexpr int CPU_FREQUENCY = 4194304;        // 4.194304 MHz (cycles per second)
constexpr double FRAME_RATE = 59.7;           // ~59.7 Hz (frames per second)
constexpr int CYCLES_PER_FRAME = static_cast<int>(CPU_FREQUENCY / FRAME_RATE);  // ~70224 cycles

// LCD timing constants
constexpr int CYCLES_PER_SCANLINE = 456; // 456 cycles per scanline
constexpr int SCANLINES_PER_FRAME = 154; // 154 scanlines per frame
constexpr int VBLANK_SCANLINE = 144; // V-Blank starts at scanline 144

// I/O Register Addresses
constexpr uint16_t IO_IF = 0xFF0F; // Interrupt Flag Register
constexpr uint16_t IO_LY = 0xFF44; // LCD Y-Coordinate Register
constexpr uint16_t IO_IE = 0xFFFF; // Interrupt Enable Register

// Interrupt bits
constexpr uint8_t INT_VBLANK = 0x01; // Bit 0: VBlank
constexpr uint8_t INT_LCD_STAT = 0x02; // Bit 1: LCD STAT
constexpr uint8_t INT_TIMER = 0x04; // Bit 2: Timer
constexpr uint8_t INT_SERIAL = 0x08; // Bit 3: Serial
constexpr uint8_t INT_JOYPAD = 0x10; // Bit 4: Joypad

// Interrupt vectors
constexpr uint16_t INT_VECTOR_VBLANK = 0x0040;
constexpr uint16_t INT_VECTOR_LCD_STAT = 0x0048;
constexpr uint16_t INT_VECTOR_TIMER = 0x0050;
constexpr uint16_t INT_VECTOR_SERIAL = 0x0058;
constexpr uint16_t INT_VECTOR_JOYPAD = 0x0060;

struct GameBoy {
    Memory memory;
    cpu::CPU cpu;
    bool running;

    // LCD state
    int lcd_cycles; // Cycles since last LCD state change
    uint8_t ly;        // Current scanline (LY register)
    
    GameBoy();
    
    void load_rom(const std::vector<uint8_t>& rom);
    
    // Execute one instruction, return cycles taken
    int step();
    
    // Execute one frame worth of cycles
    int step_frame();

    void update_lcd(int cycles);
    
    // Handle interrupts
    void handle_interrupts();
    
    // Main emulation loop
    void run();
};
