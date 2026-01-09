#include "gameboy.h"

GameBoy::GameBoy() : running(false) {}

void GameBoy::load_rom(const std::vector<uint8_t>& rom) {
    memory.load_rom(rom);
}

int GameBoy::step() {
    return cpu.execute_instruction(memory);
}

int GameBoy::step_frame() {
    int cycles_executed = 0;
    while (cycles_executed < CYCLES_PER_FRAME) {
        int cycles = step();
        if (cycles < 0) {
            running = false;
            return cycles; // Error occurred
        }
        cycles_executed += cycles;
        
        // TODO: Update other components (PPU, timers, APU) with cycles
    }
    return cycles_executed;
}

void GameBoy::run() {
    running = true;
    while (running) {
        int cycles = step_frame();
        if (cycles < 0) {
            return; // Exit on error
        }
        // TODO: Render frame, handle input, etc.
    }
}
