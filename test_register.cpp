#include <cstdint>
#include <iostream>
#include <iomanip>

struct RegisterPair
{
    union
    {
        struct
        {
            uint8_t low;  
            uint8_t high; 
        };
        uint16_t pair;
    };
    RegisterPair() : pair(0) {}
};

int main() {
    RegisterPair bc;
    bc.pair = 0x0400;  // Same as Tetris sets
    
    std::cout << "Initial BC: 0x" << std::hex << std::setw(4) << std::setfill('0') << bc.pair << std::endl;
    std::cout << "B (high): 0x" << std::setw(2) << (int)bc.high << std::endl;
    std::cout << "C (low):  0x" << std::setw(2) << (int)bc.low << std::endl;
    std::cout << "Expected: B=0x04, C=0x00" << std::endl << std::endl;
    
    // Decrement
    bc.pair--;
    std::cout << "After DEC BC:" << std::endl;
    std::cout << "BC: 0x" << std::hex << std::setw(4) << bc.pair << std::endl;
    std::cout << "B (high): 0x" << std::setw(2) << (int)bc.high << std::endl;
    std::cout << "C (low):  0x" << std::setw(2) << (int)bc.low << std::endl;
    std::cout << "Expected: B=0x03, C=0xFF" << std::endl << std::endl;
    
    // Test B OR C
    uint8_t a = bc.high;
    a |= bc.low;
    std::cout << "B OR C = 0x" << std::hex << (int)a << " (should be non-zero)" << std::endl;
    
    return 0;
}
