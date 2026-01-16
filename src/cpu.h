#pragma once
#include <cstdint>
#include <vector>

// Debug flag - can be defined via CMake with -DGBEMU_DEBUG=ON
// Or uncomment the line below to always enable it:
// #define GBEMU_DEBUG

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
#ifdef GBEMU_DEBUG
        uint64_t instructions_executed; // Total instructions executed (8 bytes) - debug only
#endif
        
        // CPU registers - direct members for cleaner access (2 bytes each)
        RegisterPair bc; // Access as bc.high (B), bc.low (C), or bc.pair (BC)
        RegisterPair de; // Access as de.high (D), de.low (E), or de.pair (DE)
        RegisterPair hl; // Access as hl.high (H), hl.low (L), or hl.pair (HL)
        RegisterPair af; // Access as af.high (A), af.low (F), or af.pair (AF)
        
        uint16_t sp; // Stack pointer
        uint16_t pc; // Program counter

        bool ime; // Interrupt Master Enable Flag (1 byte)
        bool ime_scheduled; // Delayed IME enable (1 byte)
        
        CPU();
        
        void print_state();
        void unimplemented_instruction(uint8_t opcode, const std::vector<uint8_t>& rom);
        int nop();
        int jp_a16(uint16_t addr, bool condition = true);
        int xor_a(uint8_t value, int length = 1, int cycles = 4);
        int ld_rr_n16(uint16_t& dest, uint16_t value);
        int ld_r_n8(uint8_t& dest, uint8_t value);
        int ld_hlp_a(Memory& memory, bool increment);
        int dec_r(uint8_t& reg);
        int jr_e8(int8_t offset, bool condition = true);
        int di();
        int ei();
        int ldh(Memory& memory, uint8_t offset, bool to_memory, int length = 1, int cycles = 8);
        int cp_a(uint8_t value, int length = 1, int cycles = 4);
        
        // Execute one instruction, return cycles taken
        int execute_instruction(Memory& memory);
    };
}
