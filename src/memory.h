#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

enum class MBCType 
    {
        None,
        MBC1,
        MBC2,
        MBC3,
        MBC5
    };

struct MBC 
{
    MBCType type = MBCType::None;
    // Shared registers
    bool     ram_enabled = false;
    uint16_t rom_bank = 1;
    uint8_t  ram_bank = 0;
    bool     banking_mode = false;
    
    //MBC3 RTC
    uint8_t rtc[5]         = {};
    uint8_t rtc_latched[5] = {};
    uint8_t latch_reg      = 0xFF;
    
};

struct Memory {
    // ROM data (loaded from cartridge)
    MBC mbc;
    std::vector<uint8_t> rom;
    std::vector<uint8_t> ext_ram;
    
    // Internal Game Boy memory
    uint8_t vram[0x2000];      // $8000-$9FFF: Video RAM
    uint8_t wram[0x2000];      // $C000-$DFFF: Work RAM
    uint8_t oam[0xA0];         // $FE00-$FE9F: Sprite Attribute Table
    uint8_t io[0x80];          // $FF00-$FF7F: I/O Registers
    uint8_t hram[0x7F];        // $FF80-$FFFE: High RAM
    uint8_t ie_register;       // $FFFF: Interrupt Enable
    
    uint16_t num_rom_banks = 2;
    uint8_t num_ram_banks = 0;
    bool has_battery = false;
    bool has_rtc = false;

    // Joypad state
    uint8_t joypad_state;      // Current button states (0 = pressed, 1 = released)

    // Timer state
    int div_counter;           // Internal 16-bit divider counter; DIV register = upper byte
    int tima_cycles;           // Cycle accumulator for TIMA increments

    Memory();

    // Advance timer counters by the given number of CPU cycles
    void tick_timers(int cycles);
    
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