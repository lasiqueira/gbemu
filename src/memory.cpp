#include "memory.h"
#include <print>

Memory::Memory() : ie_register(0), joypad_state(0xFF) {
    std::memset(vram, 0, sizeof(vram));
    std::memset(wram, 0, sizeof(wram));
    std::memset(oam, 0, sizeof(oam));
    std::memset(io, 0, sizeof(io));
    std::memset(hram, 0, sizeof(hram));
}

void Memory::load_rom(const std::vector<uint8_t>& rom_data) {
    rom = rom_data;
}

uint8_t Memory::read(uint16_t addr) const {
    // ROM: $0000-$7FFF (32KB for simple games like Tetris)
    if (addr < 0x8000) {
        if (addr < rom.size()) {
            return rom[addr];
        }
        return 0xFF; // Return 0xFF for unmapped ROM
    }
    
    // VRAM: $8000-$9FFF
    if (addr < 0xA000) {
        return vram[addr - 0x8000];
    }
    
    // External RAM: $A000-$BFFF (not used in Tetris)
    if (addr < 0xC000) {
        return 0xFF; // No external RAM for now
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
    // ROM: $0000-$7FFF (read-only, writes ignored for simple ROMs)
    if (addr < 0x8000) {
        // For now, ignore writes to ROM area
        // (MBC would handle banking here)
        return;
    }
    
    // VRAM: $8000-$9FFF
    if (addr < 0xA000) {
        vram[addr - 0x8000] = value;
        return;
    }
    
    // External RAM: $A000-$BFFF (not used in Tetris)
    if (addr < 0xC000) {
        return; // Ignore for now
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
