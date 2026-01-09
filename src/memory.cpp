#include "memory.h"

Memory::Memory() : ie_register(0) {
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
        io[addr - 0xFF00] = value;
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
