#include <cstdint>

struct Channel
{
    int timer = 0;
    int volume = 0;
    bool enabled = false;
};

struct APU 
{
    int frame_seq = 0;
    Channel channel1; // Square wave with sweep
    Channel channel2; // Square wave without sweep
    Channel channel3; // Wave output
    Channel channel4; // Noise generator
    
    void step(int cycles, Memory& memory);
};

