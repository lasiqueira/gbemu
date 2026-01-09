#pragma once
#include <cstdint>
#include <vector>

// Forward declaration
struct Memory;

namespace cpu
{
    // Flag bit positions
    constexpr uint8_t FLAG_ZERO = 7;
    constexpr uint8_t FLAG_SUBTRACT = 6;
    constexpr uint8_t FLAG_HALF_CARRY = 5;
    constexpr uint8_t FLAG_CARRY = 4;

    // Flag helper functions
    inline bool get_flag(uint8_t flags, uint8_t bit) { return (flags >> bit) & 1; }
    inline void set_flag(uint8_t& flags, uint8_t bit, bool value) { 
        flags = value ? (flags | (1 << bit)) : (flags & ~(1 << bit)); 
    }

    // Register pair struct using union for 8-bit/16-bit access
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

    struct CPU
    {
        // CPU registers - direct members for cleaner access
        RegisterPair bc; // Access as bc.high (B), bc.low (C), or bc.pair (BC)
        RegisterPair de; // Access as de.high (D), de.low (E), or de.pair (DE)
        RegisterPair hl; // Access as hl.high (H), hl.low (L), or hl.pair (HL)
        RegisterPair af; // Access as af.high (A), af.low (F), or af.pair (AF)
        
        uint16_t sp; // Stack pointer
        uint16_t pc; // Program counter
        
        CPU();
        
        void print_state();
        void unimplemented_instruction(uint8_t opcode, const std::vector<uint8_t>& rom);
        int nop();
        
        // Execute one instruction, return cycles taken
        int execute_instruction(Memory& memory);
    };
}
