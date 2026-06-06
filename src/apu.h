#include <cstdint>

struct PulseChannel 
{
    uint8_t nrx0 = 0; // sweep for ch1, unused for ch2
    uint8_t nrx1 = 0; // duty and length
    uint8_t nrx2 = 0; // envelope and volume
    uint8_t nrx3 = 0; // period low (write only)
    uint8_t nrx4 = 0; // period high and control

    bool enabled = false; // Whether the channel is currently producing sound

};

struct WaveChannel 
{
    uint8_t nr30 = 0; // DAC enable
    uint8_t nr31 = 0; // length
    uint8_t nr32 = 0; // output level
    uint8_t nr33 = 0; // period low (write only)
    uint8_t nr34 = 0; // period high and control
    
    bool enabled = false; // Whether the channel is currently producing sound
    uint8_t wave_pattern[16] = {}; // 32 4-bit samples stored in 16 bytes
};

struct NoiseChannel 
{
    uint8_t nr41 = 0; // length (write only)
    uint8_t nr42 = 0; // envelope and volume
    uint8_t nr43 = 0; // frequency and randomness
    uint8_t nr44 = 0; // control
    
    bool enabled = false; // Whether the channel is currently producing sound
};

struct APU 
{
    uint8_t nr52 = 0; // Audio Master control
    uint8_t nr51 = 0; // Sound Panning
    uint8_t nr50 = 0; // Volume control and VIN panning

    PulseChannel ch1;
    PulseChannel ch2;
    WaveChannel ch3;
    NoiseChannel ch4;


    uint8_t read_register(uint16_t addr);
    void write_register(uint16_t addr, uint8_t value);
};

//TODO enable/disable channel - in write_register? elsewhere?

//TODO extracting the properties from each register - in an update function?