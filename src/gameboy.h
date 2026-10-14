#pragma once
#include <cstdint>
#include <vector>
#include "constants.h"
#include "cpu.h"
#include "memory.h"
#include "ppu.h"
#include "apu.h"

enum class Model { DMG, CGB };

struct GameBoy {
    Memory memory;
    CPU cpu;
    PPU ppu;
    APU apu;
    Model requested_model = Model::DMG; // what the user asked for
    Model model = Model::DMG; // resolved at load_rom time
    bool model_is_auto = true;
    bool cgb_mode = false; // model == Model::CGB
    bool dmg_compat = false; // CGB hardware running a DMG cartridge

    bool running = false;
    
    void reset();
    
    void load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path);
    
    // Execute one instruction, return cycles taken
    int step();
    
    // Execute one frame worth of cycles
    int step_frame();
    
    // Handle interrupts
    void handle_interrupts();

    void on_memory_cycle(int cycles);

    void set_cartridge_mode();

    void apply_post_boot_state();
};
