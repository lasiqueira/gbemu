#include "apu.h"

void APU::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
        case 0xFF10: ch1.nrx0 = value; break;
        case 0xFF11: ch1.nrx1 = value; break;
        case 0xFF12: ch1.nrx2 = value; break;
        case 0xFF13: ch1.nrx3 = value; break;
        case 0xFF14: ch1.nrx4 = value; break;
        case 0xFF16: ch2.nrx1 = value; break;
        case 0xFF17: ch2.nrx2 = value; break;
        case 0xFF18: ch2.nrx3 = value; break;
        case 0xFF19: ch2.nrx4 = value; break;
        case 0xFF1A: ch3.nr30 = value; break;
        case 0xFF1B: ch3.nr31 = value; break;
        case 0xFF1C: ch3.nr32 = value; break;
        case 0xFF1D: ch3.nr33 = value; break;
        case 0xFF1E: ch3.nr34 = value; break;
        case 0xFF20: ch4.nr41 = value; break;
        case 0xFF21: ch4.nr42 = value; break;
        case 0xFF22: ch4.nr43 = value; break;
        case 0xFF23: ch4.nr44 = value; break;
        case 0xFF24: nr50 = value; break;
        case 0xFF25: nr51 = value; break;
        case 0xFF26: nr52 = value & 0x80; break; // Only bit 7 is writable
    }
}

uint8_t APU::read_register(uint16_t addr) {
    switch (addr) {
        case 0xFF10: return ch1.nrx0;
        case 0xFF11: return ch1.nrx1;
        case 0xFF12: return ch1.nrx2;
        case 0xFF13: return 0; // NR13 is write-only
        case 0xFF14: return ch1.nrx4;
        case 0xFF16: return ch2.nrx1;
        case 0xFF17: return ch2.nrx2;
        case 0xFF18: return 0; // NR23 is write-only
        case 0xFF19: return ch2.nrx4;
        case 0xFF1A: return ch3.nr30;
        case 0xFF1B: return ch3.nr31;
        case 0xFF1C: return ch3.nr32;
        case 0xFF1D: return 0; // NR33 is write-only
        case 0xFF1E: return ch3.nr34;
        case 0xFF20: return ch4.nr41;
        case 0xFF21: return ch4.nr42;
        case 0xFF22: return ch4.nr43;
        case 0xFF23: return ch4.nr44;
        case 0xFF24: return nr50;
        case 0xFF25: return nr51;
        case 0xFF26: return nr52 | (ch1.enabled << 7) | (ch2.enabled << 6) | (ch3.enabled << 5) | (ch4.enabled << 4);
        default: return 0xFF; // Unused addresses typically read as 0xFF
    }
}