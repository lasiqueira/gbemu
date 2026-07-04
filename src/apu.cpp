#include "apu.h"
#include "memory.h"
#include "constants.h"


void APU::step(int cycles, Memory& memory) {
    
    sample_counter += static_cast<float>(cycles);
    while(sample_counter >= CYCLES_PER_SAMPLE) {
        sample_counter -= CYCLES_PER_SAMPLE;
        
        sample_buffer.push_back(0); // Left channel sample (placeholder)
        sample_buffer.push_back(0); // Right channel sample (placeholder)

    }
    if(!master_enabled) {
        return; // APU is disabled, do nothing
    }

    cycle_counter += cycles;


    // Frame sequencer: 8 steps cycling at 512 Hz = one step every 8192 cycles
    while (cycle_counter >= 8192) {
        cycle_counter -= 8192;
        frame_seq_step(); // advances internal step 0→1→2→...→7→0
    }
}

void APU::on_register_write(uint16_t addr, uint8_t value) {
    switch(addr) {
        case IO_NR52: {
            bool powered = value & 0x80;
            if(!powered) {
                master_enabled = false;
                channel1.enabled = false;
                channel1.silent = true;
                channel2.enabled = false;
                channel2.silent = true;
                channel3.enabled = false;
                channel3.silent = true;
                channel4.enabled = false;
                channel4.silent = true;
            } else {
                master_enabled = true;
            }
        } break;
       default: break;
    }
}

uint8_t APU::on_register_read(uint16_t addr) {
  switch(addr) {
        case IO_NR52: {
            uint8_t val = 0;
            if(master_enabled) val |= 0x80;
            if(channel1.enabled && !channel1.silent) val |= 0x01;
            if(channel2.enabled && !channel2.silent) val |= 0x02;
            if(channel3.enabled && !channel3.silent) val |= 0x04;
            if(channel4.enabled && !channel4.silent) val |= 0x08;
            return val;
        } break;
       default: break;
    }
    return 0xFF;
}

void APU::frame_seq_step() {
    
    switch (frame_seq_counter) {
        case 0: clock_length(); break;
        case 2: clock_length(); clock_sweep(); break;
        case 4: clock_length(); break;
        case 6: clock_length(); clock_sweep(); break;
        case 7: clock_envelope(); break;
    }
   
    frame_seq_counter = (frame_seq_counter + 1) % 8;
}

void APU::on_wave_ram_write(uint16_t offset, uint8_t value) {
    if (offset < 16) {
        channel3.wave_ram[offset] = value;
    }
}

void APU::clock_length() {
    //TODO
}

void APU::clock_sweep() {
    //TODO
}

void APU::clock_envelope() {
    //TODO
}