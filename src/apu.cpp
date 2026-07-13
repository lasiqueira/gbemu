#include "apu.h"
#include "memory.h"
#include "constants.h"


void APU::step(int cycles, Memory& memory)
{
    channel1.step(cycles);
    channel2.step(cycles);
    channel3.step(cycles);
    channel4.step(cycles);

    sample_counter += cycles * AUDIO_SAMPLE_RATE;
    while (sample_counter >= static_cast<uint32_t>(CPU_FREQUENCY))
    {
        sample_counter -= CPU_FREQUENCY;
        
        int ch1_sample = channel1.sample();
        int ch2_sample = channel2.sample();
        int ch3_sample = channel3.sample(wave_ram);
        int ch4_sample = channel4.sample();

        // NR51: left:  bit 4 = CH1, bit 5 = CH2, bit 6 = CH3, bit 7 = CH4
        //       right: bit 0 = CH1, bit 1 = CH2, bit 2 = CH3, bit 3 = CH4
        int left = ((nr51 & 0x10) ? ch1_sample : 0) + ((nr51 & 0x20) ? ch2_sample : 0) + ((nr51 & 0x40) ? ch3_sample : 0) + ((nr51 & 0x80) ? ch4_sample : 0);
        int right = ((nr51 & 0x01) ? ch1_sample : 0) + ((nr51 & 0x02) ? ch2_sample : 0) + ((nr51 & 0x04) ? ch3_sample : 0) + ((nr51 & 0x08) ? ch4_sample : 0);

        int left_volume = ((nr50 >> 4) & 0x07) + 1; // 3 bits for left volume
        int right_volume = (nr50 & 0x07) + 1;       // 3 bits for right volume

        sample_buffer.push_back(static_cast<int16_t>(left * left_volume * AUDIO_SAMPLE_SCALING)); // Left channel sample. 
        sample_buffer.push_back(static_cast<int16_t>(right * right_volume * AUDIO_SAMPLE_SCALING)); // Right channel sample.

    }
    if (!master_enabled)
    {
        return; // APU is disabled, do nothing
    }

    cycle_counter += cycles;

    // Frame sequencer: 8 steps cycling at 512 Hz = one step every 8192 cycles
    while (cycle_counter >= FRAME_SEQ_CYCLES)
    {
        cycle_counter -= FRAME_SEQ_CYCLES;
        frame_seq_step(); // advances internal step 0→1→2→...→7→0
    }
}

void APU::on_register_write(uint16_t addr, uint8_t value)
{
    switch (addr)
    {
        // APU level registers
        case IO_NR52:
        {
            bool powered = value & 0x80;
            if (!powered)
            {
                master_enabled = false;
                channel1 = {};
                channel2 = {};
                channel3 = {};
                channel4 = {};
            }
            else
            {
                master_enabled = true;
            }
        }  break;
        
        case IO_NR51:
        {
             nr51 = value;           
        } break;
        
        case IO_NR50:
        {
            nr50 = value;
        } break;

        // Channel 1 Sweep register
        case IO_NR10:
        {
            channel1.sweep_pace = (value >> 4) & 0x07;
            channel1.sweep_negate = (value & 0x08) != 0;
            channel1.sweep_step = value & 0x07;
        } break;

        case IO_NR11:
        {
            channel1.duty = (value >> 6) & 0x03;
            channel1.length_value = value & 0x3F;
        } break;

        case IO_NR12:
        {
            channel1.envelope.initial_vol = (value >> 4) & 0x0F;
            channel1.dac_enabled = (value & 0xF8) != 0;
            if (!channel1.dac_enabled)
            {
                channel1.enabled = false;
            }
            channel1.envelope.env_add = (value & 0x08) != 0;
            channel1.envelope.env_pace = value & 0x07;
        } break;

        case IO_NR13:
        {
            channel1.period_value = (channel1.period_value & 0x0700) | value;
        } break;

        case IO_NR14:
        {
            channel1.period_value = (channel1.period_value & 0x00FF) | ((value & 0x07) << 8);
            channel1.length_enabled = (value & 0x40) != 0;
            bool trigger = (value & 0x80) != 0;
            if (trigger)
            {
                if (channel1.dac_enabled) channel1.enabled = true;
                channel1.period_timer = (APU_PERIOD_MAX - channel1.period_value) * 4; // Timer counts down every 4 cycles
                channel1.envelope.current_vol = channel1.envelope.initial_vol;
                channel1.envelope.env_counter = channel1.envelope.env_pace;

                channel1.shadow_period = channel1.period_value; // Initialize shadow period for sweep
                channel1.sweep_counter = (channel1.sweep_pace == 0) ? SWEEP_TIMER_DEFAULT : channel1.sweep_pace; // Reset sweep counter
                channel1.sweep_enabled = (channel1.sweep_pace != 0) || (channel1.sweep_step != 0); // Enable sweep if pace or step is non-zero
                if (channel1.length_counter == 0)
                {
                    channel1.length_counter = 64 - channel1.length_value; // Length counter is 64 - NR11 value
                }
            }
        } break;

        // Channel 2 registers
         case IO_NR21:
        {
            channel2.duty = (value >> 6) & 0x03;
            channel2.length_value = value & 0x3F;
        } break;

        case IO_NR22:
        {
            channel2.envelope.initial_vol = (value >> 4) & 0x0F;
            channel2.dac_enabled = (value & 0xF8) != 0;
            if (!channel2.dac_enabled)
            {
                channel2.enabled = false;
            }
            channel2.envelope.env_add = (value & 0x08) != 0;
            channel2.envelope.env_pace = value & 0x07;
        } break;

        case IO_NR23:
        {
            channel2.period_value = (channel2.period_value & 0x0700) | value;
        } break;

        case IO_NR24:
        {
            channel2.period_value = (channel2.period_value & 0x00FF) | ((value & 0x07) << 8);
            channel2.length_enabled = (value & 0x40) != 0;

            bool trigger = (value & 0x80) != 0;
            if (trigger)
            {
                if (channel2.dac_enabled) channel2.enabled = true;
                channel2.period_timer = (APU_PERIOD_MAX - channel2.period_value) * 4; // Timer counts down every 4 cycles
                channel2.envelope.current_vol = channel2.envelope.initial_vol;
                channel2.envelope.env_counter = channel2.envelope.env_pace;
            }
        } break;

        // Channel 3 registers
        case IO_NR30:
        {
            channel3.dac_enabled = (value & 0x80) != 0;
            if (!channel3.dac_enabled)
            {
                channel3.enabled = false;
            }
        } break;

        case IO_NR31:
        {
            channel3.length_value = value;
        } break;

        case IO_NR32:
        {
            channel3.volume = (value >> 5) & 0x03;
        } break;

        case IO_NR33:
        {
            channel3.period_value = (channel3.period_value & 0x0700) | value;
        } break;

        case IO_NR34:
        {
            channel3.period_value = (channel3.period_value & 0x00FF) | ((value & 0x07) << 8);
            channel3.length_enabled = (value & 0x40) != 0;
            bool trigger = (value & 0x80) != 0;
            if (trigger)
            {
                if (channel3.dac_enabled) channel3.enabled = true;
                channel3.period_timer = (APU_PERIOD_MAX - channel3.period_value) * 2; // Timer counts down every 2 cycles
                channel3.wave_pos = 0; // Reset wave position
                if (channel3.length_counter == 0)
                {
                    channel3.length_counter = 256 - channel3.length_value; // Length counter is 256 - NR31 value
                }
            }
        } break;

        // Channel 4 registers
        case IO_NR41:
        {
            channel4.length_value = value & 0x3F;
        } break;

        case IO_NR42:
        {
            channel4.envelope.initial_vol = (value >> 4) & 0x0F;
            channel4.dac_enabled = (value & 0xF8) != 0;
            if (!channel4.dac_enabled)
            {
                channel4.enabled = false;
            }
            channel4.envelope.env_add = (value & 0x08) != 0;
            channel4.envelope.env_pace = value & 0x07;
        } break;

        case IO_NR43:
        {
            channel4.clock_div = value & 0x07;
            channel4.clock_shift = (value >> 4) & 0x0F;
            channel4.lfsr_width = (value & 0x08) ? 7 : 15; // Bit 3 determines LFSR width
        } break;

        case IO_NR44:
        {
            channel4.length_enabled = (value & 0x40) != 0;
            bool trigger = (value & 0x80) != 0;
            if (trigger)
            {
                if (channel4.dac_enabled) channel4.enabled = true;
                channel4.envelope.env_counter = channel4.envelope.env_pace;
                channel4.envelope.current_vol = channel4.envelope.initial_vol;
                channel4.lfsr = APU_LFSR_INIT; // Reset LFSR to all 1s
                channel4.period_timer = (channel4.clock_div == 0 ? 8 : channel4.clock_div * 16) << channel4.clock_shift;
                if (channel4.length_counter == 0)
                {
                    channel4.length_counter = 64 - channel4.length_value; // Length counter is 64 - NR41 value
                }
            }
        } break;

       default: break;
    }
}

uint8_t APU::on_register_read(uint16_t addr)
{
    switch (addr)
    {
        // APU level registers
        case IO_NR52:
        {
            uint8_t val = 0;
            if (master_enabled) val |= 0x80;
            if (channel1.enabled) val |= 0x01;
            if (channel2.enabled) val |= 0x02;
            if (channel3.enabled) val |= 0x04;
            if (channel4.enabled) val |= 0x08;
            val |= 0x70; // Bits 4-6 are always read as 1
            return val;
        }
        case IO_NR51: return nr51;
        case IO_NR50: return nr50;
        
        // Channel 1 registers
        case IO_NR10: return  0x80 | ((channel1.sweep_pace << 4) | (channel1.sweep_negate ? 0x08 : 0) | channel1.sweep_step);
        case IO_NR11: return (channel1.duty << 6) | 0x3F; //only upper 2 bits are readable, lower 6 bits are write-only
        case IO_NR12: return (channel1.envelope.initial_vol << 4) | (channel1.envelope.env_add ? 0x08 : 0) | channel1.envelope.env_pace;
        case IO_NR13: return 0xFF; // NR13 is write-only, return 0xFF
        case IO_NR14: return 0xBF | (channel1.length_enabled ? 0x40 : 0);
        
        // Channel 2 registers
        case IO_NR21: return (channel2.duty << 6) |0x3F; //only upper 2 bits are readable, lower 6 bits are write-only
        case IO_NR22: return (channel2.envelope.initial_vol << 4) | (channel2.envelope.env_add ? 0x08 : 0) | channel2.envelope.env_pace;
        case IO_NR23: return 0xFF; // NR23 is write-only, return 0xFF
        case IO_NR24: return 0xBF | (channel2.length_enabled ? 0x40 : 0);
        
        // Channel 3 registers
        case IO_NR30: return (channel3.dac_enabled ? 0x80 : 0) | 0x7F; // Bit 7 is DAC enable, bits 0-6 are read as 1
        case IO_NR31: return 0xFF; // NR31 is write-only, return 0xFF
        case IO_NR32: return ((channel3.volume & 0x03) << 5) | 0x9F; // Bits 5-6 are volume, bits 0-4 are read as 1
        case IO_NR33: return 0xFF; // NR33 is write-only, return 0xFF
        case IO_NR34: return 0xBF | (channel3.length_enabled ? 0x40 : 0);
        
        // Channel 4 registers
        case IO_NR41: return 0xFF; // NR41 is write-only, return 0xFF
        case IO_NR42: return (channel4.envelope.initial_vol << 4) | (channel4.envelope.env_add ? 0x08 : 0) | channel4.envelope.env_pace;
        case IO_NR43: return (channel4.clock_shift << 4) | (channel4.lfsr_width == 7 ? 0x08 : 0) | channel4.clock_div;
        case IO_NR44: return  0xBF |(channel4.length_enabled ? 0x40 : 0);

        default: break;
    }
    return 0xFF;
}

void APU::frame_seq_step()
{
    
    switch (frame_seq_counter)
    {
        case 0: clock_length(); break;
        case 2: clock_length(); clock_sweep(); break;
        case 4: clock_length(); break;
        case 6: clock_length(); clock_sweep(); break;
        case 7: clock_envelope(); break;
    }
   
    frame_seq_counter = (frame_seq_counter + 1) % 8;
}

void APU::on_wave_ram_write(uint16_t offset, uint8_t value)
{
    if (offset < 16)
    {
        wave_ram[offset] = value;
    }
}

void Channel::clock_length()
{
    if (length_enabled && length_counter > 0)
    {
        if (--length_counter == 0) enabled = false;
    }
}

void APU::clock_length()
{
    channel1.clock_length();
    channel2.clock_length();
    channel3.clock_length();
    channel4.clock_length();
}

void APU::clock_sweep()
{
    channel1.clock_sweep();
}

void APU::clock_envelope()
{
    channel1.envelope.clock_envelope();
    channel2.envelope.clock_envelope();
    channel4.envelope.clock_envelope();
}


static constexpr uint8_t DUTY_TABLE[4] = 
{
    0b00000001, // 12.5% duty
    0b00000011, // 25% duty
    0b00001111, // 50% duty
    0b11111100  // 75% duty
};

int SquareChannel::sample() const
{
    if (!enabled || !dac_enabled) return 0;
    bool high = (DUTY_TABLE[duty] >> duty_pos) & 1;
        return high ? envelope.current_vol : 0;
}

void Envelope::clock_envelope()
{
    if (env_pace == 0) return; // No envelope operation if pace is 0
    
    --env_counter;
    if (env_counter == 0)
    {
        env_counter = env_pace;
        if(env_add && current_vol < 15)
        {
            current_vol++;
        }
        else if (!env_add && current_vol > 0)
        {
            current_vol--;
        }
    }
}

void SquareChannelWithSweep::clock_sweep()
{
    --sweep_counter;
    if(sweep_counter > 0) return; // Not time to clock sweep yet

    sweep_counter = (sweep_pace == 0) ? SWEEP_TIMER_DEFAULT : sweep_pace; // Reset sweep counter

    if (!sweep_enabled || sweep_pace == 0) return; // timer reloads but no update

    int delta = shadow_period >> sweep_step;
    int new_freq = shadow_period + (sweep_negate ? -delta : +delta);

    if(new_freq >= APU_PERIOD_MAX) // Frequency overflow, disable channel
    {
        enabled = false;
        sweep_enabled = false;
        return;
    }

    shadow_period = (uint16_t) new_freq;
    period_value = (uint16_t) new_freq;
}

static constexpr uint8_t WAVE_RAM_SHIFT[4] = 
{
    4, // mute
    0, // 100%
    1, // 50%
    2  // 25%
};

int WaveChannel::sample(const uint8_t* wave_ram) const
{
    if (!enabled || !dac_enabled) return 0;

    uint8_t wave_byte = wave_ram[wave_pos >> 1]; // Each byte contains two 4-bit samples
    uint8_t sample_nibble = (wave_pos & 1) == 0 ? (wave_byte >> 4) : (wave_byte & 0x0F);

    return (sample_nibble >> WAVE_RAM_SHIFT[volume]); // Scale sample based on volume setting
}

int NoiseChannel::sample() const
{
    if (!enabled || !dac_enabled) return 0;
    return (~lfsr & 0x01) ? envelope.current_vol : 0; // Output current volume if LFSR bit 0 is 0
}

void SquareChannel::step(int cycles)
{
    if (!enabled) return;
    period_timer -= cycles;
    while (period_timer <= 0)
    {
        period_timer += (APU_PERIOD_MAX - period_value) * 4; // Timer counts down every 4 cycles
        duty_pos = (duty_pos + 1) & 7; // Cycle through duty positions
    }
}

void WaveChannel::step(int cycles)
{
    if (!enabled) return;
    period_timer -= cycles;
    while (period_timer <= 0)
    {
        period_timer += (APU_PERIOD_MAX - period_value) * 2; // Timer counts down every 2 cycles
        wave_pos = (wave_pos + 1) & 31; // Cycle through wave positions (32 samples)
    }
}

void NoiseChannel::step(int cycles)
{
    if (!enabled) return;
    period_timer -= cycles;
    while (period_timer <= 0)
    {
        int divisor = clock_div == 0 ? 8 : clock_div * 16;
        period_timer += divisor << clock_shift; // Timer counts down based on clock shift and divisor

        uint8_t xor_bit = ((lfsr & 0x01) ^ ((lfsr >> 1) & 0x01));
        lfsr = (lfsr >> 1) | (xor_bit << 14); // Shift LFSR and insert new bit at position 14

        if (lfsr_width == 7)
        {
            lfsr = (lfsr & ~0x40) | (xor_bit << 6); // Set bit 6 for 7-bit LFSR
        }
    }
}