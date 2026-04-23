#include "memory.h"
#include <print>
#include <cstdio>

//TODO maybe initialize in the struct?
Memory::Memory() : ie_register(0), joypad_state(0xFF), div_counter(0), tima_cycles(0) {
    std::memset(vram, 0, sizeof(vram));
    std::memset(wram, 0, sizeof(wram));
    std::memset(oam, 0, sizeof(oam));
    std::memset(io, 0, sizeof(io));
    std::memset(hram, 0, sizeof(hram));
}

void Memory::load_rom(const std::vector<uint8_t>& rom_data) {
    rom = rom_data;
    mbc = MBC{}; // Reset MBC state
    has_battery = false;
    has_rtc = false;

    uint8_t cart_type = rom[0x147];
    uint8_t rom_size = rom[0x148];
    uint8_t ram_size = rom[0x149];

    // set MBC type and flags
    switch(cart_type) {
        case 0x00: mbc.type = MBCType::None; break;// ROM ONLY
        
        case 0x01: mbc.type = MBCType::MBC1; break;// MBC1
        case 0x02: mbc.type = MBCType::MBC1; break; // MBC1+RAM
        case 0x03: mbc.type = MBCType::MBC1; has_battery = true; break;// MBC1+RAM+BATT
        
        case 0x05: mbc.type = MBCType::MBC2; break;// MBC2
        case 0x06: mbc.type = MBCType::MBC2; has_battery = true; break;// MBC2+BATTERY
        
        case 0x0F: mbc.type = MBCType::MBC3; has_rtc = true; has_battery = true; break;// MBC3+RTC+BATT
        case 0x10: mbc.type = MBCType::MBC3; has_rtc = true; has_battery = true; break;// MBC3+RTC+RAM+BATT
        case 0x11: mbc.type = MBCType::MBC3; break;// MBC3
        case 0x12: mbc.type = MBCType::MBC3; break;// MBC3+RAM
        case 0x13: mbc.type = MBCType::MBC3; has_battery = true; break;// MBC3+RAM+BATT

        case 0x19: mbc.type = MBCType::MBC5; break;// MBC5
        case 0x1A: mbc.type = MBCType::MBC5; break;// MBC5+RAM
        case 0x1B: mbc.type = MBCType::MBC5; has_battery = true; break;// MBC5+RAM+BATT
        case 0x1C: mbc.type = MBCType::MBC5; break;// MBC5+RUMBLE
        case 0x1D: mbc.type = MBCType::MBC5; break;// MBC5+RUMBLE+RAM
        case 0x1E: mbc.type = MBCType::MBC5; has_battery = true; break;// MBC5+RUMBLE+RAM+BATT  
        //TODO rest of MBCs
    }

    // set ROM size
    num_rom_banks = 2 << rom_size; // 0=32KB(2 banks), 1=64KB(4 banks), 2=128KB(8 banks), etc.

    // set RAM size
    switch(ram_size) {
        case 0x00: num_ram_banks = 0; break; // No RAM
        case 0x01: num_ram_banks = 0; break; // unused
        case 0x02: num_ram_banks = 1; break; // 8KB RAM
        case 0x03: num_ram_banks = 4; break; // 32KB RAM (4 banks of 8KB each)
        case 0x04: num_ram_banks = 16; break; // 128KB RAM (16 banks of 8KB each)
        case 0x05: num_ram_banks = 8; break; // 64KB RAM (8 banks of 8KB each)
    }
    
    if (mbc.type == MBCType::MBC2) {
        ext_ram.resize(512, 0);
    } else {
        ext_ram.resize(num_ram_banks * 0x2000, 0);
    }
}

uint8_t Memory::read(uint16_t addr) const {
    if(addr < 0x4000) {
        if(mbc.type == MBCType::MBC1 && mbc.banking_mode) {
            uint32_t bank = (mbc.ram_bank << 5) % num_rom_banks;
            return rom[bank * 0x4000 + addr];
        }
        return rom[addr];
    }

    if(addr < 0x8000) {
        uint32_t bank;
        
        if(mbc.type == MBCType::MBC1) {
            bank = ((mbc.ram_bank << 5) | mbc.rom_bank) % num_rom_banks;
        } else {
            bank = mbc.rom_bank % num_rom_banks;
        }

        return rom[bank * 0x4000 + (addr - 0x4000)];
    }
    
    // VRAM: $8000-$9FFF
    if (addr < 0xA000) {
        return vram[addr - 0x8000];
    }
    
    // External RAM: $A000-$BFFF
    if (addr < 0xC000) {
        if(!mbc.ram_enabled) return 0xFF;
        
        if(mbc.type == MBCType::MBC2) {
            return ext_ram[addr & 0x1FF] | 0xF0; // upper 4 bits undefined, return as 1s
        }

        if(mbc.type == MBCType::MBC3 && mbc.ram_bank >= 0x08) {
            if(has_rtc) {
                return mbc.rtc_latched[mbc.ram_bank - 0x08];
            }

            return 0xFF;
        }

        if(!ext_ram.empty()) {
            uint8_t bank = (mbc.type == MBCType::MBC1 && !mbc.banking_mode) ? 0 : mbc.ram_bank;
            uint32_t offset = bank * 0x2000 + (addr - 0xA000);
            return ext_ram[offset % ext_ram.size()];
        }

        return 0xFF;
    }
    
    // Work RAM: $C000-$DFFF
    if (addr < 0xE000) {
        return wram[addr - 0xC000];
    }
    
    // Echo RAM: $E000-$FDFF (mirror of $C000-$DDFF)
    if (addr < 0xFE00) {
        return wram[addr - 0xE000];
    }
    
    // OAM: $FE00-$FE9F
    if (addr < 0xFEA0) {
        return oam[addr - 0xFE00];
    }
    
    // Prohibited area: $FEA0-$FEFF
    if (addr < 0xFF00) {
        return 0xFF;
    }
    
    // I/O Registers: $FF00-$FF7F
    if (addr < 0xFF80) {
        // Joypad register (0xFF00)
        if (addr == 0xFF00) {
            uint8_t joyp = io[0];
            uint8_t result = 0xC0; // Bits 6-7 always set
            
            // Bit 4: Select direction keys (0 = selected)
            // Bit 5: Select button keys (0 = selected)
            if (!(joyp & 0x10)) { // Direction keys selected
                result |= 0x10;
                result |= (joypad_state >> 4) & 0x0F; // Bits 0-3: Right, Left, Up, Down
            }
            else if (!(joyp & 0x20)) { // Button keys selected
                result |= 0x20;
                result |= joypad_state & 0x0F; // Bits 0-3: A, B, Select, Start
            }
            else {
                result |= 0x30; // Both unselected
                result |= 0x0F; // All buttons released
            }
            
            return result;
        }
        
        return io[addr - 0xFF00];
    }
    
    // High RAM: $FF80-$FFFE
    if (addr < 0xFFFF) {
        return hram[addr - 0xFF80];
    }
    
    // Interrupt Enable: $FFFF
    return ie_register;
}

void Memory::write(uint16_t addr, uint8_t value) {
    
    // RAM ENABLE: $0000-$1FFF (MBC1/MBC3), $0000-$3FFF (MBC2)
    if (addr < 0x2000) {
        if(mbc.type == MBCType::MBC2 && (addr & 0x100)) return; // bit 8 set = not RAM enable
        mbc.ram_enabled = (value & 0x0F) == 0x0A; // 0x0A enables RAM, any other value disables it
        return;
    }
    
    // ROM BANK NUMBER: $2000-$3FFF (MBC1/MBC3), $0000-$3FFF (MBC2)
    if (addr < 0x4000){
        switch (mbc.type) {
            case MBCType::MBC1:
                mbc.rom_bank = value & 0x1F; 
                if (mbc.rom_bank == 0) mbc.rom_bank = 1;
                break;
            case MBCType::MBC2:
                if (addr & 0x100) {  // only when bit 8 of address is set
                    mbc.rom_bank = value & 0x0F; 
                    if (mbc.rom_bank == 0) mbc.rom_bank = 1; 
                }
                break;
            case MBCType::MBC3:
                mbc.rom_bank = value & 0x7F;
                if (mbc.rom_bank == 0) mbc.rom_bank = 1;
                break;
            case MBCType::MBC5:
                if(addr < 0x3000) {
                    mbc.rom_bank = (mbc.rom_bank & 0x100) | value; // low 8 bits
                } else {
                    mbc.rom_bank = (mbc.rom_bank & 0xFF) | ((value & 0x01) << 8); // bit 9
                }
                break;
            default:
                break;    
        }
        return;
    }

    // RAM BANK NUMBER: $4000-$5FFF (MBC1), $4000-$5FFF (MBC3), not used in MBC2
    if (addr < 0x6000) {
        switch(mbc.type) {
            case MBCType::MBC1: mbc.ram_bank = value & 0x03; break;
            case MBCType::MBC3: mbc.ram_bank = value; break;
            case MBCType::MBC5: mbc.ram_bank = value & 0x0F; break;
            default: break;
        }
        return;
    }

    // Banking Mode (MBC1) / Latch Clock (MBC3
    if (addr < 0x8000) {
        switch (mbc.type) {
            case MBCType::MBC1:
                mbc.banking_mode = value & 0x01;
                break;
            case MBCType::MBC3:
                if (mbc.latch_reg == 0x00 && value == 0x01) {
                    std::memcpy(mbc.rtc_latched, mbc.rtc, 5);
                }
                mbc.latch_reg = value;
                break;
            default:
                break;
        }
        return;
    }

    // VRAM: $8000-$9FFF
    if (addr < 0xA000) {
        vram[addr - 0x8000] = value;
        return;
    }
    
    // External RAM: $A000-$BFFF (not used in Tetris)
    if (addr < 0xC000) {
        if (!mbc.ram_enabled) return;
        if (mbc.type == MBCType::MBC2) {
            ext_ram[addr & 0x1FF] = value & 0x0F;
        } else if (mbc.type == MBCType::MBC3 && mbc.ram_bank >= 0x08 && has_rtc) {
            mbc.rtc[mbc.ram_bank - 0x08] = value;
        } else if (!ext_ram.empty()) {
            uint8_t bank = (mbc.type == MBCType::MBC1 && !mbc.banking_mode) ? 0 : mbc.ram_bank;
            uint32_t offset = bank * 0x2000 + (addr - 0xA000);
            ext_ram[offset % ext_ram.size()] = value; 
        }
        return; 
    }
    
    // Work RAM: $C000-$DFFF
    if (addr < 0xE000) {
        wram[addr - 0xC000] = value;
        return;
    }
    
    // Echo RAM: $E000-$FDFF (mirror of $C000-$DDFF)
    if (addr < 0xFE00) {
        wram[addr - 0xE000] = value;
        return;
    }
    
    // OAM: $FE00-$FE9F
    if (addr < 0xFEA0) {
        oam[addr - 0xFE00] = value;
        return;
    }
    
    // Prohibited area: $FEA0-$FEFF
    if (addr < 0xFF00) {
        return;
    }
    
    // I/O Registers: $FF00-$FF7F
    if (addr < 0xFF80) {
        // Joypad register (0xFF00) - only bits 4-5 are writable
        if (addr == 0xFF00) {
            io[0] = (value & 0x30) | 0xC0; // Keep only bits 4-5, set bits 6-7
        }
        // Serial transfer control (0xFF02)
        // Bit 7: Transfer Start, Bit 0: Clock Select (0=external/slave, 1=internal/master)
        // Only auto-complete when using internal clock (master mode, 0x81).
        // External clock (slave, 0x80) requires a real linked partner to clock the transfer;
        // without one it stalls forever — completing it would fake a second Game Boy.
        else if (addr == 0xFF02) {
            io[0x02] = value;
            if ((value & 0x81) == 0x81) { // Transfer start + internal clock
                char c = static_cast<char>(io[0x01]);
                putchar(c);
                fflush(stdout);
                io[0x02] &= ~0x80;  // Clear transfer-start bit (transfer complete)
                io[0x0F] |= 0x08;   // Request serial interrupt (IF bit 3)
            }
        }
        // DIV register (0xFF04): any write resets the internal counter
        else if (addr == 0xFF04) {
            div_counter = 0;
            io[0x04] = 0;
        }
        else if (addr == 0xFF40) {
            io[0x40] = value;
        }
        // DMA transfer (0xFF46) - OAM DMA
        else if (addr == 0xFF46) {
            // DMA copies 160 bytes from XX00-XX9F to FE00-FE9F
            uint16_t source = value * 0x100;
            for (int i = 0; i < 0xA0; i++) {
                oam[i] = read(source + i);
            }
            io[0x46] = value;
        }
        else {
            io[addr - 0xFF00] = value;
        }
        return;
    }
    
    // High RAM: $FF80-$FFFE
    if (addr < 0xFFFF) {
        hram[addr - 0xFF80] = value;
        return;
    }
    
    // Interrupt Enable: $FFFF
    ie_register = value;
}

uint16_t Memory::read_word(uint16_t addr) const {
    uint8_t low = read(addr);
    uint8_t high = read(addr + 1);
    return (high << 8) | low;
}

void Memory::write_word(uint16_t addr, uint16_t value) {
    write(addr, value & 0xFF);
    write(addr + 1, (value >> 8) & 0xFF);
}

void Memory::tick_timers(int cycles) {
    // Update DIV (internal 16-bit counter, upper byte exposed at 0xFF04)
    div_counter = (div_counter + cycles) & 0xFFFF;
    io[0x04] = static_cast<uint8_t>(div_counter >> 8);

    // Update TIMA if timer is enabled (TAC bit 2)
    uint8_t tac = io[0x07]; // 0xFF07
    if (!(tac & 0x04)) return;

    // TIMA increment thresholds (CPU cycles per tick)
    static const int tima_thresholds[4] = {1024, 16, 64, 256};
    int threshold = tima_thresholds[tac & 0x03];

    tima_cycles += cycles;
    while (tima_cycles >= threshold) {
        tima_cycles -= threshold;
        uint8_t tima = io[0x05]; // 0xFF05
        tima++;
        if (tima == 0) {
            io[0x05] = io[0x06]; // Reset TIMA to TMA (0xFF06)
            io[0x0F] |= 0x04;   // Request timer interrupt (IF bit 2)
        } else {
            io[0x05] = tima;
        }
    }
}

