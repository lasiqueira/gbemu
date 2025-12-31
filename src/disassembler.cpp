#include <print>
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

        std::string format(uint16_t address, const uint8_t *bytes) const
        {
            // Build full instruction string
            std::string full_instr = mnemonic;
            if (!operands.empty())
            {
                full_instr += " " + operands;
            }

            // Format operands with actual values from bytes
            std::string formatted_operands = operands;
            if (length == 2 && formatted_operands.find("n8") != std::string::npos)
            {
                formatted_operands = std::format("${:02X}", bytes[1]);
            }
            else if (length == 2 && formatted_operands.find("e8") != std::string::npos)
            {
                formatted_operands = std::format("${:02X}", static_cast<int8_t>(bytes[1]));
            }
            else if (length == 3 && formatted_operands.find("n16") != std::string::npos)
            {
                formatted_operands = std::format("${:04X}", bytes[1] | (bytes[2] << 8));
            }
            else if (length == 3 && formatted_operands.find("a16") != std::string::npos)
            {
                formatted_operands = std::format("${:04X}", bytes[1] | (bytes[2] << 8));
            }

            // Build instruction with formatted operands
            std::string display_instr = mnemonic;
            if (!formatted_operands.empty() && formatted_operands != operands)
            {
                display_instr += " " + formatted_operands;
            }
            else if (!operands.empty())
            {
                display_instr += " " + operands;
            }

            // Build flags string
            std::string flags_str;
            if (!affected_flags.empty())
            {
                for (size_t i = 0; i < affected_flags.size(); ++i)
                {
                    flags_str += affected_flags[i];
                    if (i < affected_flags.size() - 1)
                        flags_str += ",";
                }
            }
            else
            {
                flags_str = "-";
            }

            // Build cycles string
            std::string cycles_str;
            if (cycles_branch_taken != cycles)
            {
                cycles_str = std::format("{}/{}", cycles_branch_taken, cycles);
            }
            else
            {
                cycles_str = std::format("{}", cycles);
            }

            return std::format("{:04X}  {:20s}  {:8s}  {:3d}  {:5s}",
                               address,
                               display_instr,
                               flags_str,
                               length,
                               cycles_str);
        }

        void print(uint16_t address, const uint8_t *bytes) const
        {
            std::println("{}", format(address, bytes));
        }
    };

    // Forward declaration
    Instruction decode_cb_instruction(uint8_t opcode);

    // Decode instruction from opcode
    Instruction decode_instruction(const uint8_t* bytes, size_t available = 3)
    {
        uint8_t opcode = bytes[0];
        switch (opcode)
        {
        case 0x00: return {"NOP", "", {}, 1, 4, 4};
        case 0x01: return {"LD", "BC, n16", {}, 3, 12, 12};
        case 0x02: return {"LD", "(BC), A", {}, 1, 8, 8};
        case 0x03: return {"INC", "BC", {}, 1, 8, 8};
        case 0x04: return {"INC", "B", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x05: return {"DEC", "B", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x06: return {"LD", "B, n8", {}, 2, 8, 8};
        case 0x07: return {"RLCA", "", {"0", "0", "0", "C"}, 1, 4, 4};
        case 0x08: return {"LD", "(a16), SP", {}, 3, 20, 20};
        case 0x09: return {"ADD", "HL, BC", {"-", "0", "H", "C"}, 1, 8, 8};
        case 0x0A: return {"LD", "A, (BC)", {}, 1, 8, 8};
        case 0x0B: return {"DEC", "BC", {}, 1, 8, 8};
        case 0x0C: return {"INC", "C", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x0D: return {"DEC", "C", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x0E: return {"LD", "C, n8", {}, 2, 8, 8};
        case 0x0F: return {"RRCA", "", {"0", "0", "0", "C"}, 1, 4, 4};
        case 0x10: return {"STOP", "0", {}, 2, 4, 4};
        case 0x11: return {"LD", "DE, n16", {}, 3, 12, 12};
        case 0x12: return {"LD", "(DE), A", {}, 1, 8, 8};
        case 0x13: return {"INC", "DE", {}, 1, 8, 8};
        case 0x14: return {"INC", "D", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x15: return {"DEC", "D", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x16: return {"LD", "D, n8", {}, 2, 8, 8};
        case 0x17: return {"RLA", "", {"0", "0", "0", "C"}, 1, 4, 4};
        case 0x18: return {"JR", "e8", {}, 2, 12, 12};
        case 0x19: return {"ADD", "HL, DE", {"-", "0", "H", "C"}, 1, 8, 8};
        case 0x1A: return {"LD", "A, (DE)", {}, 1, 8, 8};
        case 0x1B: return {"DEC", "DE", {}, 1, 8, 8};
        case 0x1C: return {"INC", "E", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x1D: return {"DEC", "E", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x1E: return {"LD", "E, n8", {}, 2, 8, 8};
        case 0x1F: return {"RRA", "", {"0", "0", "0", "C"}, 1, 4, 4};
        case 0x20: return {"JR", "NZ, e8", {}, 2, 8, 12};
        case 0x21: return {"LD", "HL, n16", {}, 3, 12, 12};
        case 0x22: return {"LD", "(HL+), A", {}, 1, 8, 8};
        case 0x23: return {"INC", "HL", {}, 1, 8, 8};
        case 0x24: return {"INC", "H", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x25: return {"DEC", "H", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x26: return {"LD", "H, n8", {}, 2, 8, 8};
        case 0x27: return {"DAA", "", {"Z", "-", "H", "C"}, 1, 4, 4};
        case 0x28: return {"JR", "Z, e8", {}, 2, 8, 12};
        case 0x29: return {"ADD", "HL, HL", {"-", "0", "H", "C"}, 1, 8, 8};
        case 0x2A: return {"LD", "A, (HL+)", {}, 1, 8, 8};
        case 0x2B: return {"DEC", "HL", {}, 1, 8, 8};
        case 0x2C: return {"INC", "L", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x2D: return {"DEC", "L", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x2E: return {"LD", "L, n8", {}, 2, 8, 8};
        case 0x2F: return {"CPL", "", {"-", "1", "1", "-"}, 1, 4, 4};
        case 0x30: return {"JR", "NC, e8", {}, 2, 8, 12};
        case 0x31: return {"LD", "SP, n16", {}, 3, 12, 12};
        case 0x32: return {"LD", "(HL-), A", {}, 1, 8, 8};
        case 0x33: return {"INC", "SP", {}, 1, 8, 8};
        case 0x34: return {"INC", "(HL)", {"Z", "0", "H", "-"}, 1, 12, 12};
        case 0x35: return {"DEC", "(HL)", {"Z", "1", "H", "-"}, 1, 12, 12};
        case 0x36: return {"LD", "(HL), n8", {}, 2, 12, 12};
        case 0x37: return {"SCF", "", {"-", "0", "0", "1"}, 1, 4, 4};
        case 0x38: return {"JR", "C, e8", {}, 2, 8, 12};
        case 0x39: return {"ADD", "HL, SP", {"-", "0", "H", "C"}, 1, 8, 8};
        case 0x3A: return {"LD", "A, (HL-)", {}, 1, 8, 8};
        case 0x3B: return {"DEC", "SP", {}, 1, 8, 8};
        case 0x3C: return {"INC", "A", {"Z", "0", "H", "-"}, 1, 4, 4};
        case 0x3D: return {"DEC", "A", {"Z", "1", "H", "-"}, 1, 4, 4};
        case 0x3E: return {"LD", "A, n8", {}, 2, 8, 8};
        case 0x3F: return {"CCF", "", {"-", "0", "0", "C"}, 1, 4, 4};
        case 0x40: return {"LD", "B, B", {}, 1, 4, 4};
        case 0x41: return {"LD", "B, C", {}, 1, 4, 4};
        case 0x42: return {"LD", "B, D", {}, 1, 4, 4};
        case 0x43: return {"LD", "B, E", {}, 1, 4, 4};
        case 0x44: return {"LD", "B, H", {}, 1, 4, 4};
        case 0x45: return {"LD", "B, L", {}, 1, 4, 4};
        case 0x46: return {"LD", "B, (HL)", {}, 1, 8, 8};
        case 0x47: return {"LD", "B, A", {}, 1, 4, 4};
        case 0x48: return {"LD", "C, B", {}, 1, 4, 4};
        case 0x49: return {"LD", "C, C", {}, 1, 4, 4};
        case 0x4A: return {"LD", "C, D", {}, 1, 4, 4};
        case 0x4B: return {"LD", "C, E", {}, 1, 4, 4};
        case 0x4C: return {"LD", "C, H", {}, 1, 4, 4};
        case 0x4D: return {"LD", "C, L", {}, 1, 4, 4};
        case 0x4E: return {"LD", "C, (HL)", {}, 1, 8, 8};
        case 0x4F: return {"LD", "C, A", {}, 1, 4, 4};
        case 0x50: return {"LD", "D, B", {}, 1, 4, 4};
        case 0x51: return {"LD", "D, C", {}, 1, 4, 4};
        case 0x52: return {"LD", "D, D", {}, 1, 4, 4};
        case 0x53: return {"LD", "D, E", {}, 1, 4, 4};
        case 0x54: return {"LD", "D, H", {}, 1, 4, 4};
        case 0x55: return {"LD", "D, L", {}, 1, 4, 4};
        case 0x56: return {"LD", "D, (HL)", {}, 1, 8, 8};
        case 0x57: return {"LD", "D, A", {}, 1, 4, 4};
        case 0x58: return {"LD", "E, B", {}, 1, 4, 4};
        case 0x59: return {"LD", "E, C", {}, 1, 4, 4};
        case 0x5A: return {"LD", "E, D", {}, 1, 4, 4};
        case 0x5B: return {"LD", "E, E", {}, 1, 4, 4};
        case 0x5C: return {"LD", "E, H", {}, 1, 4, 4};
        case 0x5D: return {"LD", "E, L", {}, 1, 4, 4};
        case 0x5E: return {"LD", "E, (HL)", {}, 1, 8, 8};
        case 0x5F: return {"LD", "E, A", {}, 1, 4, 4};
        case 0x60: return {"LD", "H, B", {}, 1, 4, 4};
        case 0x61: return {"LD", "H, C", {}, 1, 4, 4};
        case 0x62: return {"LD", "H, D", {}, 1, 4, 4};
        case 0x63: return {"LD", "H, E", {}, 1, 4, 4};
        case 0x64: return {"LD", "H, H", {}, 1, 4, 4};
        case 0x65: return {"LD", "H, L", {}, 1, 4, 4};
        case 0x66: return {"LD", "H, (HL)", {}, 1, 8, 8};
        case 0x67: return {"LD", "H, A", {}, 1, 4, 4};
        case 0x68: return {"LD", "L, B", {}, 1, 4, 4};
        case 0x69: return {"LD", "L, C", {}, 1, 4, 4};
        case 0x6A: return {"LD", "L, D", {}, 1, 4, 4};
        case 0x6B: return {"LD", "L, E", {}, 1, 4, 4};
        case 0x6C: return {"LD", "L, H", {}, 1, 4, 4};
        case 0x6D: return {"LD", "L, L", {}, 1, 4, 4};
        case 0x6E: return {"LD", "L, (HL)", {}, 1, 8, 8};
        case 0x6F: return {"LD", "L, A", {}, 1, 4, 4};
        case 0x70: return {"LD", "(HL), B", {}, 1, 8, 8};
        case 0x71: return {"LD", "(HL), C", {}, 1, 8, 8};
        case 0x72: return {"LD", "(HL), D", {}, 1, 8, 8};
        case 0x73: return {"LD", "(HL), E", {}, 1, 8, 8};
        case 0x74: return {"LD", "(HL), H", {}, 1, 8, 8};
        case 0x75: return {"LD", "(HL), L", {}, 1, 8, 8};
        case 0x76: return {"HALT", "", {}, 1, 4, 4};
        case 0x77: return {"LD", "(HL), A", {}, 1, 8, 8};
        case 0x78: return {"LD", "A, B", {}, 1, 4, 4};
        case 0x79: return {"LD", "A, C", {}, 1, 4, 4};
        case 0x7A: return {"LD", "A, D", {}, 1, 4, 4};
        case 0x7B: return {"LD", "A, E", {}, 1, 4, 4};
        case 0x7C: return {"LD", "A, H", {}, 1, 4, 4};
        case 0x7D: return {"LD", "A, L", {}, 1, 4, 4};
        case 0x7E: return {"LD", "A, (HL)", {}, 1, 8, 8};
        case 0x7F: return {"LD", "A, A", {}, 1, 4, 4};
        case 0x80: return {"ADD", "A, B", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x81: return {"ADD", "A, C", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x82: return {"ADD", "A, D", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x83: return {"ADD", "A, E", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x84: return {"ADD", "A, H", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x85: return {"ADD", "A, L", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x86: return {"ADD", "A, (HL)", {"Z", "0", "H", "C"}, 1, 8, 8};
        case 0x87: return {"ADD", "A, A", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x88: return {"ADC", "A, B", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x89: return {"ADC", "A, C", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x8A: return {"ADC", "A, D", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x8B: return {"ADC", "A, E", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x8C: return {"ADC", "A, H", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x8D: return {"ADC", "A, L", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x8E: return {"ADC", "A, (HL)", {"Z", "0", "H", "C"}, 1, 8, 8};
        case 0x8F: return {"ADC", "A, A", {"Z", "0", "H", "C"}, 1, 4, 4};
        case 0x90: return {"SUB", "B", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x91: return {"SUB", "C", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x92: return {"SUB", "D", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x93: return {"SUB", "E", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x94: return {"SUB", "H", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x95: return {"SUB", "L", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x96: return {"SUB", "(HL)", {"Z", "1", "H", "C"}, 1, 8, 8};
        case 0x97: return {"SUB", "A", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x98: return {"SBC", "A, B", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x99: return {"SBC", "A, C", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x9A: return {"SBC", "A, D", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x9B: return {"SBC", "A, E", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x9C: return {"SBC", "A, H", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x9D: return {"SBC", "A, L", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0x9E: return {"SBC", "A, (HL)", {"Z", "1", "H", "C"}, 1, 8, 8};
        case 0x9F: return {"SBC", "A, A", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xA0: return {"AND", "B", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA1: return {"AND", "C", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA2: return {"AND", "D", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA3: return {"AND", "E", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA4: return {"AND", "H", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA5: return {"AND", "L", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA6: return {"AND", "(HL)", {"Z", "0", "1", "0"}, 1, 8, 8};
        case 0xA7: return {"AND", "A", {"Z", "0", "1", "0"}, 1, 4, 4};
        case 0xA8: return {"XOR", "B", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xA9: return {"XOR", "C", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xAA: return {"XOR", "D", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xAB: return {"XOR", "E", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xAC: return {"XOR", "H", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xAD: return {"XOR", "L", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xAE: return {"XOR", "(HL)", {"Z", "0", "0", "0"}, 1, 8, 8};
        case 0xAF: return {"XOR", "A", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB0: return {"OR", "B", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB1: return {"OR", "C", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB2: return {"OR", "D", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB3: return {"OR", "E", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB4: return {"OR", "H", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB5: return {"OR", "L", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB6: return {"OR", "(HL)", {"Z", "0", "0", "0"}, 1, 8, 8};
        case 0xB7: return {"OR", "A", {"Z", "0", "0", "0"}, 1, 4, 4};
        case 0xB8: return {"CP", "B", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xB9: return {"CP", "C", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xBA: return {"CP", "D", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xBB: return {"CP", "E", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xBC: return {"CP", "H", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xBD: return {"CP", "L", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xBE: return {"CP", "(HL)", {"Z", "1", "H", "C"}, 1, 8, 8};
        case 0xBF: return {"CP", "A", {"Z", "1", "H", "C"}, 1, 4, 4};
        case 0xC0: return {"RET", "NZ", {}, 1, 8, 20};
        case 0xC1: return {"POP", "BC", {}, 1, 12, 12};
        case 0xC2: return {"JP", "NZ, a16", {}, 3, 12, 16};
        case 0xC3: return {"JP", "a16", {}, 3, 16, 16};
        case 0xC4: return {"CALL", "NZ, a16", {}, 3, 12, 24};
        case 0xC5: return {"PUSH", "BC", {}, 1, 16, 16};
        case 0xC6: return {"ADD", "A, n8", {"Z", "0", "H", "C"}, 2, 8, 8};
        case 0xC7: return {"RST", "00H", {}, 1, 16, 16};
        case 0xC8: return {"RET", "Z", {}, 1, 8, 20};
        case 0xC9: return {"RET", "", {}, 1, 16, 16};
        case 0xCA: return {"JP", "Z, a16", {}, 3, 12, 16};
        case 0xCB: return (available > 1) ? decode_cb_instruction(bytes[1]) : Instruction{"UNKNOWN", "", {}, 1, 4, 4};
        case 0xCC: return {"CALL", "Z, a16", {}, 3, 12, 24};
        case 0xCD: return {"CALL", "a16", {}, 3, 24, 24};
        case 0xCE: return {"ADC", "A, n8", {"Z", "0", "H", "C"}, 2, 8, 8};
        case 0xCF: return {"RST", "08H", {}, 1, 16, 16};
        case 0xD0: return {"RET", "NC", {}, 1, 8, 20};
        case 0xD1: return {"POP", "DE", {}, 1, 12, 12};
        case 0xD2: return {"JP", "NC, a16", {}, 3, 12, 16};
        case 0xD4: return {"CALL", "NC, a16", {}, 3, 12, 24};
        case 0xD5: return {"PUSH", "DE", {}, 1, 16, 16};
        case 0xD6: return {"SUB", "n8", {"Z", "1", "H", "C"}, 2, 8, 8};
        case 0xD7: return {"RST", "10H", {}, 1, 16, 16};
        case 0xD8: return {"RET", "C", {}, 1, 8, 20};
        case 0xD9: return {"RETI", "", {}, 1, 16, 16};
        case 0xDA: return {"JP", "C, a16", {}, 3, 12, 16};
        case 0xDC: return {"CALL", "C, a16", {}, 3, 12, 24};
        case 0xDE: return {"SBC", "A, n8", {"Z", "1", "H", "C"}, 2, 8, 8};
        case 0xDF: return {"RST", "18H", {}, 1, 16, 16};
        case 0xE0: return {"LDH", "(a8), A", {}, 2, 12, 12};
        case 0xE1: return {"POP", "HL", {}, 1, 12, 12};
        case 0xE2: return {"LD", "(C), A", {}, 1, 8, 8};
        case 0xE5: return {"PUSH", "HL", {}, 1, 16, 16};
        case 0xE6: return {"AND", "n8", {"Z", "0", "1", "0"}, 2, 8, 8};
        case 0xE7: return {"RST", "20H", {}, 1, 16, 16};
        case 0xE8: return {"ADD", "SP, e8", {"0", "0", "H", "C"}, 2, 16, 16};
        case 0xE9: return {"JP", "(HL)", {}, 1, 4, 4};
        case 0xEA: return {"LD", "(a16), A", {}, 3, 16, 16};
        case 0xEE: return {"XOR", "n8", {"Z", "0", "0", "0"}, 2, 8, 8};
        case 0xEF: return {"RST", "28H", {}, 1, 16, 16};
        case 0xF0: return {"LDH", "A, (a8)", {}, 2, 12, 12};
        case 0xF1: return {"POP", "AF", {}, 1, 12, 12};
        case 0xF2: return {"LD", "A, (C)", {}, 1, 8, 8};
        case 0xF3: return {"DI", "", {}, 1, 4, 4};
        case 0xF5: return {"PUSH", "AF", {}, 1, 16, 16};
        case 0xF6: return {"OR", "n8", {"Z", "0", "0", "0"}, 2, 8, 8};
        case 0xF7: return {"RST", "30H", {}, 1, 16, 16};
        case 0xF8: return {"LD", "HL, SP+e8", {"0", "0", "H", "C"}, 2, 12, 12};
        case 0xF9: return {"LD", "SP, HL", {}, 1, 8, 8};
        case 0xFA: return {"LD", "A, (a16)", {}, 3, 16, 16};
        case 0xFB: return {"EI", "", {}, 1, 4, 4};
        case 0xFE: return {"CP", "n8", {"Z", "1", "H", "C"}, 2, 8, 8};
        case 0xFF: return {"RST", "38H", {}, 1, 16, 16};
        default: return {"UNKNOWN", "", {}, 1, 4, 4};
        }
    }

    Instruction decode_cb_instruction(uint8_t opcode)
{
    uint8_t reg = opcode & 0x07;
    const char* reg_names[] = {"B", "C", "D", "E", "H", "L", "(HL)", "A"};
    std::string r = reg_names[reg];
    
    // Cycles: most are 8, except (HL) operations are 16
    uint8_t cycles = (reg == 6) ? 16 : 8;
    
    if (opcode < 0x08) return {"RLC", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x10) return {"RRC", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x18) return {"RL", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x20) return {"RR", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x28) return {"SLA", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x30) return {"SRA", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    if (opcode < 0x38) return {"SWAP", r, {"Z", "0", "0", "0"}, 2, cycles, cycles};
    if (opcode < 0x40) return {"SRL", r, {"Z", "0", "0", "C"}, 2, cycles, cycles};
    
    // BIT/RES/SET instructions
    uint8_t bit = (opcode >> 3) & 0x07;
    std::string operands = std::format("{}, {}", bit, r);
    
    if (opcode < 0x80) return {"BIT", operands, {"Z", "0", "1", "-"}, 2, cycles, cycles};
    if (opcode < 0xC0) return {"RES", operands, {}, 2, cycles, cycles};
    return {"SET", operands, {}, 2, cycles, cycles};
}

    void print_disassembly(const std::vector<uint8_t> &rom, size_t start_addr = 0, size_t end_addr = 0)
    {
        std::println("Disassembly:");
        std::println("Addr  Instruction           Flags     Len  Cycles");
        std::println("----  --------------------  --------  ---  ------");

        if (end_addr == 0 || end_addr > rom.size())
        {
            end_addr = rom.size();
        }

        size_t addr = start_addr;
        while (addr < end_addr)
        {
            uint8_t bytes[3] = {0};
            size_t available = std::min(size_t(3), rom.size() - addr);
            for (size_t j = 0; j < available; ++j)
            {
                bytes[j] = rom[addr + j];
            }

            Instruction instr = decode_instruction(bytes, available);

            instr.print(addr, bytes);

            addr += instr.length;
        }
    }

    void disassemble_rom(const std::vector<uint8_t> &rom, size_t start_addr = 0x100, size_t end_addr = 0)
    {
        std::println("ROM size: {} bytes", rom.size());
        std::println("");
        print_disassembly(rom, start_addr, end_addr);
    }

    // Debug helper: print a single instruction at PC
    void print_instruction_at(const std::vector<uint8_t> &memory, uint16_t pc)
    {
        if (pc >= memory.size())
            return;

        uint8_t bytes[3] = {0};
        size_t available = std::min(size_t(3), memory.size() - pc);
        for (size_t i = 0; i < available; ++i)
        {
            bytes[i] = memory[pc + i];
        }

        Instruction instr = decode_instruction(bytes, available);

        instr.print(pc, bytes);
    }
}