#pragma once
#include <cstdint>
#include <vector>
#include "cpu.h"
#include "memory.h"

// Game Boy timing constants
constexpr int CPU_FREQUENCY = 4194304;        // 4.194304 MHz (cycles per second)
constexpr double FRAME_RATE = 59.7;           // ~59.7 Hz (frames per second)
constexpr int CYCLES_PER_FRAME = static_cast<int>(CPU_FREQUENCY / FRAME_RATE);  // ~70224 cycles

struct GameBoy {
    Memory memory;
    cpu::CPU cpu;
    bool running;
    
    GameBoy();
    
    void load_rom(const std::vector<uint8_t>& rom);
    
    // Execute one instruction, return cycles taken
    int step();
    
    // Execute one frame worth of cycles
    int step_frame();
    
    // Main emulation loop
    void run();
};
