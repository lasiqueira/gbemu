#include "cpu.h"
#include "memory.h"
#include "disassembler.h"
#include <print>

namespace cpu
{
    CPU::CPU() {
        pc = 0x100; // Game Boy starts execution at 0x100
        sp = 0xFFFE;
    }
    
    void CPU::print_state() {
        std::println("CPU State:");
        std::println("A: {:04X}  B: {:04X}  C: {:04X}  D: {:04X}  E: {:04X}  H: {:04X}  L: {:04X}  SP: {:04X}  PC: {:04X}",
                     af.high, bc.high, bc.low, de.high, de.low, hl.high, hl.low, sp, pc);
        std::println("Flags: Z={} N={} H={} C={}",
                     get_flag(af.low, FLAG_ZERO),
                     get_flag(af.low, FLAG_SUBTRACT),
                     get_flag(af.low, FLAG_HALF_CARRY),
                     get_flag(af.low, FLAG_CARRY));
    }
    
    void CPU::unimplemented_instruction(uint8_t opcode, const std::vector<uint8_t>& rom) {
        std::println("Unimplemented instruction: 0x{:02X} at PC: 0x{:04X}", opcode, pc);
        std::println("");
        disassembler::print_instruction_at(rom, pc);
        std::println("");  // Blank line for readability
        print_state();
    }
    
    int CPU::nop() {
        pc += 1;
        return 4; // NOP takes 4 cycles
    }
    int CPU::jp_a16(uint16_t addr, bool condition) {
        if (condition) {
            pc = addr;
            return 16; // JP takes 16 cycles if taken
        } else {
            pc += 3; // Move past the instruction and operands
            return 12; // JP takes 12 cycles if not taken
        }
    }

    int CPU::xor_a(uint8_t value, int extra_pc, int extra_cycles) {
        af.high ^= value;
        
        // Set flags
        set_flag(af.low, FLAG_ZERO, af.high == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, false);
        set_flag(af.low, FLAG_CARRY, false);
        
        pc += 1 + extra_pc; // Move past the instruction and any extra bytes
        return 4 + extra_cycles; // Base cycles plus any extra
    }

    int CPU::ld_rr_n16(uint16_t& dest, uint16_t value) {
        dest = value;
        pc += 3; // Move past the instruction and operands
        return 12; // LD rr, n16 takes 12 cycles
    }
    
    int CPU::ld_r_n8(uint8_t& dest, uint8_t value) {
        dest = value;
        pc += 2; // Move past the instruction and operand
        return 8; // LD r, n8 takes 8 cycles
    }

    int CPU::ld_hlp_a(Memory& memory, bool increment) {
        memory.write(hl.pair, af.high);
        hl.pair += increment ? 1 : -1;
        pc += 1; // Move past the instruction
        return 8; // LD (HL+/-), A takes 8 cycles
    }

    // Execute one instruction, return cycles taken
    int CPU::execute_instruction(Memory& memory) {
        uint8_t opcode = memory.read(pc);
        switch (opcode) {
            case 0x00: return nop(); // NOP
            case 0x01: return ld_rr_n16(bc.pair, memory.read_word(pc + 1)); // LD BC, n16
            case 0x06: return ld_r_n8(bc.high, memory.read(pc + 1)); // LD B, n8
            case 0x0E: return ld_r_n8(bc.low, memory.read(pc + 1)); // LD C, n8
            case 0x11: return ld_rr_n16(de.pair, memory.read_word(pc + 1)); // LD DE, n16
            case 0x16: return ld_r_n8(de.high, memory.read(pc + 1)); // LD D, n8
            case 0x1E: return ld_r_n8(de.low, memory.read(pc + 1)); // LD E, n8
            case 0x21: return ld_rr_n16(hl.pair, memory.read_word(pc + 1)); // LD HL, n16
            case 0x22: return ld_hlp_a(memory, true); // LD (HL+), A
            case 0x26: return ld_r_n8(hl.high, memory.read(pc + 1)); // LD H, n8
            case 0x2E: return ld_r_n8(hl.low, memory.read(pc + 1)); // LD L, n8
            case 0x31: return ld_rr_n16(sp, memory.read_word(pc + 1)); // LD SP, n16
            case 0x32: return ld_hlp_a(memory, false); // LD (HL-), A
            case 0x3E: return ld_r_n8(af.high, memory.read(pc + 1)); // LD A, n8  
            case 0xA8: return xor_a(bc.high); // XOR B
            case 0xA9: return xor_a(bc.low);  // XOR C
            case 0xAA: return xor_a(de.high); // XOR D
            case 0xAB: return xor_a(de.low);  // XOR E
            case 0xAC: return xor_a(hl.high); // XOR H
            case 0xAD: return xor_a(hl.low);  // XOR L
            case 0xAE: return xor_a(memory.read(hl.pair), 0, 4); // XOR (HL)
            case 0xAF: return xor_a(af.high); // XOR A
            case 0xC2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_ZERO)); // JP NZ, a16
            case 0xC3: return jp_a16(memory.read_word(pc + 1)); // JP a16
            case 0xCA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_ZERO)); // JP Z, a16
            case 0xD2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_CARRY)); // JP NC, a16
            case 0xDA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_CARRY)); // JP C, a16
            case 0xEE: return xor_a(memory.read(pc + 1), 1, 4); // XOR n8
            default:
                unimplemented_instruction(opcode, memory.rom);
                return -1; // Indicate error for unimplemented instruction
        }
    }
}
