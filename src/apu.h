#pragma once
#include <cstdint>
#include <vector>

struct Memory; // Forward declaration

struct Channel
{
    int      period_timer   = 0; // must stay int — NoiseChannel stores values up to ~917504
    uint16_t length_counter = 0; // max 256 (CH3), safe
    uint8_t  length_value   = 0;
    bool     dac_enabled    = false;
    bool     enabled        = false;
    bool     length_enabled = false; // Stores raw length register value for triggering
    void clock_length();
};

struct Envelope {
    uint8_t initial_vol = 0;
    uint8_t current_vol = 0;
    uint8_t env_pace    = 0;
    uint8_t env_counter = 0;
    bool    env_add     = false;
};

struct SquareChannel : public Channel
{
    uint16_t period_value = 0;
    uint8_t duty = 0;
    uint8_t duty_pos = 0;
    Envelope envelope;

    int sample() const;
};

struct SquareChannelWithSweep : public SquareChannel
{
    uint16_t shadow_period  = 0;
    uint8_t  sweep_pace     = 0;
    uint8_t  sweep_step     = 0;
    uint8_t  sweep_counter  = 0;
    bool     sweep_negate   = false;
    bool     sweep_enabled  = false; // Track if sweep has fired
};

struct WaveChannel : public Channel
{
    uint16_t period_value = 0;
    uint8_t  volume = 0; // 0=off, 1=100%, 2=50%, 3=25%
    uint8_t  wave_pos = 0;
};

struct NoiseChannel : public Channel
{   
    uint16_t lfsr        = 0;  // 15-bit LFSR
    uint8_t  clock_shift = 0;
    uint8_t  clock_div   = 0;
    uint8_t  lfsr_width  = 15; // 15-bit or 7-bit
    Envelope envelope;
};

struct APU 
{
    uint16_t cycle_counter = 0;
    uint8_t  frame_seq_counter = 0;
    float    sample_counter = 0.0f;
    bool     master_enabled = false;
    uint8_t  nr50 = 0; // Master volume & VIN panning
    uint8_t  nr51 = 0; // Sound panning
    uint8_t  nr52 = 0; // Audio Master control

    SquareChannelWithSweep channel1; // Square wave with sweep
    SquareChannel channel2; // Square wave without sweep
    WaveChannel channel3; // Wave output
    NoiseChannel channel4; // Noise generator
    
    uint8_t wave_ram[16] = {};

    std::vector<int16_t> sample_buffer; // Output audio samples (16-bit signed)
    void step(int cycles, Memory& memory);
    void on_register_write(uint16_t addr, uint8_t value);
    uint8_t on_register_read(uint16_t addr);
    void frame_seq_step();
    void on_wave_ram_write(uint16_t offset, uint8_t value);
    void clock_length();
    void clock_sweep();
    void clock_envelope();
};

