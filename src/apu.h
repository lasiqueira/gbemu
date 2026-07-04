#pragma once
#include <cstdint>
#include <vector>

struct Memory; // Forward declaration
struct Channel
{
    bool enabled = false;
    bool silent = false;
    int length_counter = 0;
    int timer_counter = 0;
    uint8_t length_value = 0; // Stores raw length register value for triggering
};
struct Envelope 
{
    int initial_vol = 0;
    int current_vol = 0;
    int env_period = 0;
    int env_counter = 0;
    bool env_add = false;
};
struct SquareChannel : public Channel
{
    int period = 0;
    uint8_t duty = 0;
    
    Envelope envelope;
};

struct SquareChannelWithSweep : public SquareChannel
{
    int sweep_period = 0;
    int sweep_shift = 0;
    bool sweep_add = true;
    int sweep_counter = 0;
    int initial_freq = 0;
    int current_freq = 0;
    bool sweep_enabled = false; // Track if sweep has fired

};

struct WaveChannel : public Channel
{
    int period = 0;
    uint8_t volume = 0; // 0=off, 1=100%, 2=50%, 3=25%
    uint8_t wave_pos = 0;
    uint8_t wave_ram[16] = {};
};

struct NoiseChannel : public Channel
{
    int shift = 0;
    int divisor = 0;
    int lfsr = 0; // 15-bit LFSR
    int lfsr_width = 15; // 15-bit or 7-bit

    Envelope envelope;
};

struct APU 
{
    int frame_seq_counter = 0;
    int cycle_counter = 0;
    float sample_counter = 0.0f;
    bool master_enabled = false;
    uint8_t nr50 = 0; // Master volume & VIN panning
    uint8_t nr51 = 0; // Sound panning
    uint8_t nr52 = 0; // Audio Master control

    SquareChannelWithSweep channel1; // Square wave with sweep
    SquareChannel channel2; // Square wave without sweep
    WaveChannel channel3; // Wave output
    NoiseChannel channel4; // Noise generator
    
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

