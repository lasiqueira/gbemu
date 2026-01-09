#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

struct Memory {
    // ROM data (loaded from cartridge)
    std::vector<uint8_t> rom;
    
    // Internal Game Boy memory
    uint8_t vram[0x2000];      // $8000-$9FFF: Video RAM
    uint8_t wram[0x2000];      // $C000-$DFFF: Work RAM
    uint8_t oam[0xA0];         // $FE00-$FE9F: Sprite Attribute Table
    uint8_t io[0x80];          // $FF00-$FF7F: I/O Registers
    uint8_t hram[0x7F];        // $FF80-$FFFE: High RAM
    uint8_t ie_register;       // $FFFF: Interrupt Enable
    
    Memory();
    
    // Load ROM from file data
    void load_rom(const std::vector<uint8_t>& rom_data);
    
    // Read byte from memory
    uint8_t read(uint16_t addr) const;
    
    // Write byte to memory
    void write(uint16_t addr, uint8_t value);
    
    // Read 16-bit word (little-endian)
    uint16_t read_word(uint16_t addr) const;
    
    // Write 16-bit word (little-endian)
    void write_word(uint16_t addr, uint16_t value);
};