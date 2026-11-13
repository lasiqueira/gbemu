#pragma once
#include <cstdint>
#include <vector>

// APU timing and audio constants
constexpr int AUDIO_SAMPLE_RATE = 44100;
constexpr int APU_PERIOD_MAX = 2048; // Max period value for square/wave channels (2048 = 0x800)
constexpr int AUDIO_SAMPLE_SCALING = 68; // max per side is 4 ch × 15 vol × 8 NR50 = 480
constexpr uint16_t APU_LFSR_INIT = 0x7FFF; // Initial value for 15-bit LFSR in noise channel
constexpr uint8_t SWEEP_TIMER_DEFAULT = 8;
constexpr int LENGTH_MAX = 64;       // Length counter range for the pulse and noise channels
constexpr int LENGTH_MAX_WAVE = 256; // Length counter range for the wave channel
constexpr uint16_t PERIOD_HIGH_MASK = 0x0700; // Period bits 8-10 (NRx4 bits 0-2)
constexpr uint16_t PERIOD_LOW_MASK = 0x00FF;  // Period bits 0-7 (NRx3)
constexpr uint8_t NRX4_PERIOD_HIGH_BITS = 0x07;
constexpr int ENVELOPE_VOLUME_MAX = 15;
constexpr int WAVE_RAM_SIZE = 16;     // 16 bytes = 32 four-bit samples
constexpr int WAVE_SAMPLE_COUNT = 32;
constexpr uint8_t NR43_LFSR_WIDTH_BIT = 0x08; // Set = 7-bit LFSR
constexpr int LFSR_WIDTH_SHORT = 7;
constexpr int LFSR_WIDTH_LONG = 15;
constexpr int LFSR_FEED_BIT = LFSR_WIDTH_LONG - 1; // New bit is shifted in at the top of the 15-bit register
constexpr uint16_t FRAME_SEQ_DIV_BIT = 0x1000;        // DIV bit 12 falling edge clocks the 512 Hz frame sequencer
constexpr uint16_t FRAME_SEQ_DIV_BIT_DOUBLE = 0x2000; // One bit higher in double speed, since DIV runs twice as fast

// I/O AUDIO Register Addresses
constexpr uint16_t IO_NR10   = 0xFF10; // Channel 1 Sweep
constexpr uint16_t IO_NR11   = 0xFF11; // Channel 1 length timer & duty cycle
constexpr uint16_t IO_NR12   = 0xFF12; // Channel 1 volume & envelope
constexpr uint16_t IO_NR13   = 0xFF13; // Channel 1 period low [write-only]
constexpr uint16_t IO_NR14   = 0xFF14; // Channel 1 period high & control

constexpr uint16_t IO_NR21   = 0xFF16; // Channel 2 length timer & duty cycle
constexpr uint16_t IO_NR22   = 0xFF17; // Channel 2 volume & envelope
constexpr uint16_t IO_NR23   = 0xFF18; // Channel 2 period low [write-only]
constexpr uint16_t IO_NR24   = 0xFF19; // Channel 2 period high & control

constexpr uint16_t IO_NR30   = 0xFF1A; // Channel 3 DAC enable
constexpr uint16_t IO_NR31   = 0xFF1B; // Channel 3 length timer [write-only]
constexpr uint16_t IO_NR32   = 0xFF1C; // Channel 3 output level
constexpr uint16_t IO_NR33   = 0xFF1D; // Channel 3 period low [write-only]
constexpr uint16_t IO_NR34   = 0xFF1E; // Channel 3 period high & control
constexpr uint16_t IO_WAVE_RAM_START = 0xFF30; // Channel 3 wave pattern RAM (0xFF30-0xFF3F)
constexpr uint16_t IO_WAVE_RAM_END   = 0xFF3F; // End of Channel 3 wave pattern RAM

constexpr uint16_t IO_NR41   = 0xFF20; // Channel 4 length timer [write-only]
constexpr uint16_t IO_NR42   = 0xFF21; // Channel 4 volume & envelope
constexpr uint16_t IO_NR43   = 0xFF22; // Channel 4 frequency & randomness
constexpr uint16_t IO_NR44   = 0xFF23; // Channel 4 control

constexpr uint16_t IO_NR50   = 0xFF24; // Master volume & VIN panning
constexpr uint16_t IO_NR51   = 0xFF25; // Sound panning
constexpr uint16_t IO_NR52   = 0xFF26; // Audio master control

struct Memory; // Forward declaration

struct Channel
{
    int period_timer = 0; // must stay int — NoiseChannel stores values up to ~917504
    int period_reload = APU_PERIOD_MAX * 4; // Reload value for period timer (calculated from period_value)
    uint16_t length_counter = 0; // max 256 (CH3), safe
    uint8_t length_value = 0;
    bool dac_enabled = false;
    bool enabled = false;
    bool length_enabled = false; // Stores raw length register value for triggering
    void clock_length();
};

struct Envelope
{
    uint8_t initial_vol = 0;
    uint8_t current_vol = 0;
    uint8_t env_pace = 0;
    uint8_t env_counter = 0;
    bool env_add = false;
    void clock_envelope();
};

struct SquareChannel : public Channel
{
    uint16_t period_value = 0;
    uint8_t duty = 0;
    uint8_t duty_pos = 0;
    Envelope envelope;

    int sample() const;
    void step(int cycles);
};

struct SquareChannelWithSweep : public SquareChannel
{
    uint16_t shadow_period = 0;
    uint8_t sweep_pace = 0;
    uint8_t sweep_step = 0;
    uint8_t sweep_counter = 0;
    bool sweep_negate = false;
    bool sweep_enabled = false; // Track if sweep has fired
    bool negate_was_used = false; 

    void clock_sweep();
};

struct WaveChannel : public Channel
{  
    bool just_read_sample = false;
    uint16_t period_value = 0;
    uint8_t volume = 0; // 0=off, 1=100%, 2=50%, 3=25%
    uint8_t wave_pos = 0;

    int sample(const uint8_t* wave_ram) const;
    void step(int cycles);
};

struct NoiseChannel : public Channel
{
    uint16_t lfsr = 0; // 15-bit LFSR
    uint8_t clock_shift = 0;
    uint8_t clock_div = 0;
    uint8_t lfsr_width = 15; // 15-bit or 7-bit
    Envelope envelope;

    int sample() const;
    void step(int cycles);
};

struct APU
{
    uint8_t frame_seq_counter = 0;
    uint32_t sample_counter = 0; 
    bool master_enabled = false;
    uint8_t nr50 = 0; // Master volume & VIN panning
    uint8_t nr51 = 0; // Sound panning
    uint8_t nr52 = 0; // Audio Master control

    SquareChannelWithSweep channel1; // Square wave with sweep
    SquareChannel channel2; // Square wave without sweep
    WaveChannel channel3; // Wave output
    NoiseChannel channel4; // Noise generator
    
    uint8_t wave_ram[WAVE_RAM_SIZE] = {};

    bool cgb_mode = false;

    std::vector<int16_t> sample_buffer; // Output audio samples (16-bit signed)
    void step(int sys, int cycles, Memory& memory);
    void on_register_write(uint16_t addr, uint8_t value);
    uint8_t on_register_read(uint16_t addr);
    void frame_seq_step();
    void on_wave_ram_write(uint16_t offset, uint8_t value);
    uint8_t on_wave_ram_read(uint16_t offset) const;
    // Handles the three-step length-clock logic common to all NRx4 writes:
    // 1) pre-trigger extra clock on 0→1 enable transition at odd frame-seq step
    // 2) trigger reload (counter 0 → max)
    // 3) post-trigger extra clock when trigger just reloaded from 0
    void apply_length_clock(Channel& ch, bool old_len, bool triggered, uint16_t max_len);
    void clock_length();
    void clock_sweep();
    void clock_envelope();
};

