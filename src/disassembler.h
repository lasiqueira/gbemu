#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace disassembler
{
    struct Instruction
    {
        std::string mnemonic;
        std::string operands;
        std::vector<std::string> affected_flags;
        uint8_t length;
        uint8_t cycles;              // Base cycles (when branch not taken, or single value for non-conditional)
        uint8_t cycles_branch_taken; // Cycles when conditional branch/action is taken

        std::string format(uint16_t address, const uint8_t *bytes) const;
        void print(uint16_t address, const uint8_t *bytes) const;
    };

    // Function declarations
    Instruction decode_cb_instruction(uint8_t opcode);
    Instruction decode_instruction(const uint8_t* bytes, size_t available = 3);
    void print_disassembly(const std::vector<uint8_t> &rom, size_t start_addr = 0, size_t end_addr = 0);
    void disassemble_rom(const std::vector<uint8_t> &rom, size_t start_addr = 0x100, size_t end_addr = 0);
    void print_instruction_at(const std::vector<uint8_t> &memory, uint16_t pc, bool print_header = true);
}
