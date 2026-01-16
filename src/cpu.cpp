#include "cpu.h"
#include "memory.h"
#include "disassembler.h"
#include <print>

namespace cpu
{
    CPU::CPU() {
        pc = 0x100; // Game Boy starts execution at 0x100
        sp = 0xFFFE;
        ime = false;
        ime_scheduled = false;
#ifdef GBEMU_DEBUG
        instructions_executed = 0;
#endif
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
#ifdef GBEMU_DEBUG
        if (instructions_executed > 0) {
            std::println("Instructions executed before error: {}", instructions_executed);
        }
#endif
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

    int CPU::xor_a(uint8_t value, int length, int cycles) {
        af.high ^= value;
        
        // Set flags
        set_flag(af.low, FLAG_ZERO, af.high == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, false);
        set_flag(af.low, FLAG_CARRY, false);
        
        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
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

    int CPU::dec_r(uint8_t& reg) {
        reg--;
        
        // Set flags
        set_flag(af.low, FLAG_ZERO, reg == 0);
        set_flag(af.low, FLAG_SUBTRACT, true);
        set_flag(af.low, FLAG_HALF_CARRY, (reg & 0x0F) == 0x0F);
        
        pc += 1; // Move past the instruction
        return 4; // DEC r takes 4 cycles
    }

    int CPU::jr_e8(int8_t offset, bool condition) {
        pc += 2; // Move past the instruction
        if (condition) {
            pc += offset; // Apply offset
            return 12; // JR takes 12 cycles if taken
        } else {
            return 8; // JR takes 8 cycles if not taken
        }
    }

    int CPU::di() {
        ime = false;
        pc += 1; // Move past the instruction
        return 4; // DI takes 4 cycles
    }

    int CPU::ei() {
        ime_scheduled = true;
        pc += 1; // Move past the instruction
        return 4; // EI takes 4 cycles
    }

    int CPU::ldh(Memory& memory, uint8_t offset, bool to_memory, int length, int cycles) {
        if (to_memory) {
            memory.write(0xFF00 + offset, af.high);
        } else {
            af.high = memory.read(0xFF00 + offset);
        }
        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    int CPU::cp_a(uint8_t value, int length, int cycles) {
        uint8_t result = af.high - value;
        
        // Set flags
        set_flag(af.low, FLAG_ZERO, result == 0);
        set_flag(af.low, FLAG_SUBTRACT, true);
        set_flag(af.low, FLAG_HALF_CARRY, (af.high & 0x0F) < (value & 0x0F));
        set_flag(af.low, FLAG_CARRY, af.high < value);
        
        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    // Execute one instruction, return cycles taken
    int CPU::execute_instruction(Memory& memory) {
        // Handle delayed IME enable
        if (ime_scheduled) {
            ime = true;
            ime_scheduled = false;
        }
#ifdef GBEMU_DEBUG
        // Print instruction before executing and increment counter (debug only)
        disassembler::print_instruction_at(memory.rom, pc, false);
        instructions_executed++;
#endif

        uint8_t opcode = memory.read(pc);
        switch (opcode) {
            case 0x00: return nop(); // NOP
            case 0x01: return ld_rr_n16(bc.pair, memory.read_word(pc + 1)); // LD BC, n16
            case 0x05: return dec_r(bc.high); // DEC B
            case 0x06: return ld_r_n8(bc.high, memory.read(pc + 1)); // LD B, n8
            case 0x0D: return dec_r(bc.low); // DEC C
            case 0x0E: return ld_r_n8(bc.low, memory.read(pc + 1)); // LD C, n8
            case 0x11: return ld_rr_n16(de.pair, memory.read_word(pc + 1)); // LD DE, n16
            case 0x15: return dec_r(de.high); // DEC D
            case 0x16: return ld_r_n8(de.high, memory.read(pc + 1)); // LD D, n8
            case 0x18: return jr_e8(static_cast<int8_t>(memory.read(pc + 1))); // JR e8
            case 0x1D: return dec_r(de.low); // DEC E
            case 0x1E: return ld_r_n8(de.low, memory.read(pc + 1)); // LD E, n8
            case 0x20: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), !get_flag(af.low, FLAG_ZERO)); // JR NZ, e8
            case 0x21: return ld_rr_n16(hl.pair, memory.read_word(pc + 1)); // LD HL, n16
            case 0x22: return ld_hlp_a(memory, true); // LD (HL+), A
            case 0x25: return dec_r(hl.high); // DEC H
            case 0x26: return ld_r_n8(hl.high, memory.read(pc + 1)); // LD H, n8
            case 0x28: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), get_flag(af.low, FLAG_ZERO)); // JR Z, e8
            case 0x2D: return dec_r(hl.low); // DEC L
            case 0x2E: return ld_r_n8(hl.low, memory.read(pc + 1)); // LD L, n8
            case 0x30: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), !get_flag(af.low, FLAG_CARRY)); // JR NC, e8
            case 0x31: return ld_rr_n16(sp, memory.read_word(pc + 1)); // LD SP, n16
            case 0x32: return ld_hlp_a(memory, false); // LD (HL-), A
            case 0x38: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), get_flag(af.low, FLAG_CARRY)); // JR C, e8
            case 0x3D: return dec_r(af.high); // DEC A
            case 0x3E: return ld_r_n8(af.high, memory.read(pc + 1)); // LD A, n8  
            case 0xA8: return xor_a(bc.high); // XOR B
            case 0xA9: return xor_a(bc.low);  // XOR C
            case 0xAA: return xor_a(de.high); // XOR D
            case 0xAB: return xor_a(de.low);  // XOR E
            case 0xAC: return xor_a(hl.high); // XOR H
            case 0xAD: return xor_a(hl.low);  // XOR L
            case 0xAE: return xor_a(memory.read(hl.pair), 1, 8); // XOR (HL)
            case 0xAF: return xor_a(af.high); // XOR A
            case 0xB8: return cp_a(bc.high); // CP B
            case 0xB9: return cp_a(bc.low);  // CP C
            case 0xBA: return cp_a(de.high); // CP D
            case 0xBB: return cp_a(de.low);  // CP E
            case 0xBC: return cp_a(hl.high); // CP H
            case 0xBD: return cp_a(hl.low);  // CP L
            case 0xBE: return cp_a(memory.read(hl.pair), 1, 8); // CP (HL)
            case 0xBF: return cp_a(af.high); // CP A
            case 0xC2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_ZERO)); // JP NZ, a16
            case 0xC3: return jp_a16(memory.read_word(pc + 1)); // JP a16
            case 0xCA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_ZERO)); // JP Z, a16
            case 0xD2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_CARRY)); // JP NC, a16
            case 0xDA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_CARRY)); // JP C, a16
            case 0xE0: return ldh(memory, memory.read(pc + 1), true, 2, 12); // LDH (a8), A
            case 0xE2: return ldh(memory, bc.low, true); // LDH (C), A
            case 0xEE: return xor_a(memory.read(pc + 1), 2, 8); // XOR n8
            case 0xF0: return ldh(memory, memory.read(pc + 1), false, 2, 12); // LDH A, (a8)
            case 0xF2: return ldh(memory, bc.low, false); // LDH A, (C)
            case 0xF3: return di(); // DI
            case 0xFB: return ei(); // EI
            case 0xFE: return cp_a(memory.read(pc + 1), 2, 8); // CP n8
            default:
                unimplemented_instruction(opcode, memory.rom);
                return -1; // Indicate error for unimplemented instruction
        }
    }
}
