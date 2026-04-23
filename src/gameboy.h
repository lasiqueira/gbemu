#pragma once
#include <cstdint>
#include <vector>
#include "constants.h"
#include "cpu.h"
#include "memory.h"
#include "ppu.h"

struct GameBoy {
    Memory memory;
    cpu::CPU cpu;
    PPU ppu;
    bool running;
    
    GameBoy();
    
    void load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path);
    
    // Execute one instruction, return cycles taken
    int step();
    
    // Execute one frame worth of cycles
    int step_frame();
    
    // Handle interrupts
    void handle_interrupts();
};
