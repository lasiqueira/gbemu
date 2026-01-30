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
    
    int CPU::ld_r_n8(uint8_t& dest, uint8_t value, int length, int cycles) {
        dest = value;
        pc += length; // Move past the instruction and operand
        return cycles; // Return the cycle count
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

    int CPU::ld_mem_n8(Memory& memory, uint16_t addr, uint8_t value, int length, int cycles) {
        memory.write(addr, value);
        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    int CPU::ld_a_hlp(Memory& memory, bool increment) {
        af.high = memory.read(hl.pair);
        hl.pair += increment ? 1 : -1;
        pc += 1; // Move past the instruction
        return 8; // LD A, (HL+/-) takes 8 cycles
    }

    int CPU::inc_r(uint8_t& reg) {
        reg++;

        // set flags
        set_flag(af.low, FLAG_ZERO, reg == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, (reg & 0x0F) == 0x00);

        pc += 1; // Move past the instruction
        return 4; // INC r takes 4 cycles
    }

    int CPU::call_a16(Memory& memory, uint16_t addr, bool condition) {
        if(condition) {
            sp -= 2;
            // Push current PC onto stack
            memory.write_word(sp, pc + 3); // +3 to move past CALL instruction
            pc = addr;
            return 24; // CALL takes 24 cycles if taken
        } else {
            pc += 3; // Move past the instruction and operands
            return 12; // CALL takes 12 cycles if not taken
        }
    }

    int CPU::dec_rr(uint16_t& regpair) {
        regpair--;
        pc += 1; // Move past the instruction
        return 8; // DEC rr takes 8 cycles
    }

    int CPU::or_a(uint8_t value, int length, int cycles) {
        af.high |= value;
        
        // Set flags
        set_flag(af.low, FLAG_ZERO, af.high == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, false);
        set_flag(af.low, FLAG_CARRY, false);
        
        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    int CPU::ret(Memory& memory, bool condition, int cycles_if_taken, bool enable_interrupts) {
        if (condition) {
            pc = memory.read_word(sp);
            sp += 2;
            if (enable_interrupts) {
                ime = true;
            }
            return cycles_if_taken; // RET takes specified cycles if taken
        } else {
            pc += 1; // Move past the instruction
            return 8; // RET takes 8 cycles if not taken
        }
    }

    int CPU::cpl() {
        af.high = ~af.high;
        set_flag(af.low, FLAG_SUBTRACT, true);
        set_flag(af.low, FLAG_HALF_CARRY, true);
        pc += 1; // Move past the instruction
        return 4; // CPL takes 4 cycles
    }

    int CPU::and_a(uint8_t value, int length, int cycles) {
        af.high &= value;

        // Set flags
        set_flag(af.low, FLAG_ZERO, af.high == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, true);
        set_flag(af.low, FLAG_CARRY, false);

        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    int CPU::swap_r(uint8_t& reg) {
        reg = (reg << 4) | (reg >> 4);

        // Set flags
        set_flag(af.low, FLAG_ZERO, reg == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, false);
        set_flag(af.low, FLAG_CARRY, false);

        pc += 2; // Move past the instruction
        return 8; // SWAP r takes 8 cycles
    }

    int CPU::swap_mem_hl(Memory& memory) {
        uint8_t value = memory.read(hl.pair);
        value = (value << 4) | (value >> 4);

        // Set flags
        set_flag(af.low, FLAG_ZERO, value == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, false);
        set_flag(af.low, FLAG_CARRY, false);

        memory.write(hl.pair, value);
        pc += 2; // Move past the instruction
        return 16; // SWAP (HL) takes 16 cycles
    }

    int CPU::rst(Memory& memory, uint8_t addr) {
        sp -= 2;
        memory.write_word(sp, pc + 1); // +1 to move past RST instruction
        pc = addr;
        return 16; // RST takes 16 cycles
    }

    int CPU::add_a(uint8_t value, int length, int cycles) {
        uint16_t result = static_cast<uint16_t>(af.high) + static_cast<uint16_t>(value);

        // Set flags
        set_flag(af.low, FLAG_ZERO, (result & 0xFF) == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, ((af.high & 0x0F) + (value & 0x0F)) > 0x0F);
        set_flag(af.low, FLAG_CARRY, result > 0xFF);

        af.high = static_cast<uint8_t>(result & 0xFF);

        pc += length; // Move past the instruction and operands
        return cycles; // Return the cycle count
    }

    int CPU::pop_rr(Memory& memory, uint16_t& dest) {
        dest = memory.read_word(sp);
        sp += 2;
        pc += 1; // Move past the instruction
        return 12; // POP rr takes 12 cycles
    }

    int CPU::add_hl_rr(uint16_t value) {
        uint32_t result = static_cast<uint32_t>(hl.pair) + static_cast<uint32_t>(value);

        // Set flags
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, ((hl.pair & 0x0FFF) + (value & 0x0FFF)) > 0x0FFF);
        set_flag(af.low, FLAG_CARRY, result > 0xFFFF);

        hl.pair = static_cast<uint16_t>(result);

        pc += 1; // Move past the instruction
        return 8; // ADD HL, rr takes 8 cycles
    }

    int CPU::inc_rr(uint16_t& regpair) {
        regpair++;
        pc += 1; // Move past the instruction
        return 8; // INC rr takes 8 cycles
    }

    int CPU::push_rr(Memory& memory, uint16_t value) {
        sp -= 2;
        memory.write_word(sp, value);
        pc += 1; // Move past the instruction
        return 16; // PUSH rr takes 16 cycles
    }

    int CPU::jp_hl() {
        pc = hl.pair;
        return 4; // JP HL takes 4 cycles
    }

    int CPU::res(uint8_t bit, uint8_t& reg) {
        reg &= ~(1 << bit);
        pc += 2; // Move past the instruction
        return 8; // RES b, r takes 8 cycles
    }

    int CPU::res_mem_hl(Memory& memory, uint8_t bit) {
        uint8_t value = memory.read(hl.pair);
        value &= ~(1 << bit);
        memory.write(hl.pair, value);
        pc += 2; // Move past the instruction
        return 16; // RES b, (HL) takes 16 cycles
    }

    int CPU::inc_mem_hl(Memory& memory) {
        uint8_t value = memory.read(hl.pair);
        value++;

        // Set flags
        set_flag(af.low, FLAG_ZERO, value == 0);
        set_flag(af.low, FLAG_SUBTRACT, false);
        set_flag(af.low, FLAG_HALF_CARRY, (value & 0x0F) == 0x00);

        memory.write(hl.pair, value);
        pc += 1; // Move past the instruction
        return 12; // INC (HL) takes 12 cycles
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
            case 0x02: return ld_mem_n8(memory, bc.pair, af.high, 1, 8); // LD (BC), A
            case 0x03: return inc_rr(bc.pair); // INC BC
            case 0x04: return inc_r(bc.high); // INC B
            case 0x05: return dec_r(bc.high); // DEC B
            case 0x06: return ld_r_n8(bc.high, memory.read(pc + 1)); // LD B, n8
            case 0x09: return add_hl_rr(bc.pair); // ADD HL, BC
            case 0x0A: return ld_r_n8(af.high, memory.read(bc.pair), 1, 8); // LD A, (BC)
            case 0x0B: return dec_rr(bc.pair); // DEC BC
            case 0x0C: return inc_r(bc.low); // INC C
            case 0x0D: return dec_r(bc.low); // DEC C
            case 0x0E: return ld_r_n8(bc.low, memory.read(pc + 1)); // LD C, n8
            case 0x11: return ld_rr_n16(de.pair, memory.read_word(pc + 1)); // LD DE, n16
            case 0x12: return ld_mem_n8(memory, de.pair, af.high, 1, 8); // LD (DE), A
            case 0x13: return inc_rr(de.pair); // INC DE
            case 0x14: return inc_r(de.high); // INC D
            case 0x15: return dec_r(de.high); // DEC D
            case 0x16: return ld_r_n8(de.high, memory.read(pc + 1)); // LD D, n8
            case 0x19: return add_hl_rr(de.pair); // ADD HL, DE
            case 0x1A: return ld_r_n8(af.high, memory.read(de.pair), 1, 8); // LD A, (DE)
            case 0x18: return jr_e8(static_cast<int8_t>(memory.read(pc + 1))); // JR e8
            case 0x1B: return dec_rr(de.pair); // DEC DE
            case 0x1C: return inc_r(de.low); // INC E
            case 0x1D: return dec_r(de.low); // DEC E
            case 0x1E: return ld_r_n8(de.low, memory.read(pc + 1)); // LD E, n8
            case 0x20: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), !get_flag(af.low, FLAG_ZERO)); // JR NZ, e8
            case 0x21: return ld_rr_n16(hl.pair, memory.read_word(pc + 1)); // LD HL, n16
            case 0x22: return ld_hlp_a(memory, true); // LD (HL+), A
            case 0x23: return inc_rr(hl.pair); // INC HL
            case 0x24: return inc_r(hl.high); // INC H
            case 0x25: return dec_r(hl.high); // DEC H
            case 0x26: return ld_r_n8(hl.high, memory.read(pc + 1)); // LD H, n8
            case 0x28: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), get_flag(af.low, FLAG_ZERO)); // JR Z, e8
            case 0x29: return add_hl_rr(hl.pair); // ADD HL, HL
            case 0x2A: return ld_a_hlp(memory, true); // LD A, (HL+)
            case 0x2B: return dec_rr(hl.pair); // DEC HL
            case 0x2C: return inc_r(hl.low); // INC L
            case 0x2D: return dec_r(hl.low); // DEC L
            case 0x2E: return ld_r_n8(hl.low, memory.read(pc + 1)); // LD L, n8
            case 0x2F: return cpl(); // CPL
            case 0x30: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), !get_flag(af.low, FLAG_CARRY)); // JR NC, e8
            case 0x31: return ld_rr_n16(sp, memory.read_word(pc + 1)); // LD SP, n16
            case 0x32: return ld_hlp_a(memory, false); // LD (HL-), A
            case 0x33: return inc_rr(sp); // INC SP
            case 0x34: return inc_mem_hl(memory); // INC (HL)
            case 0x36: return ld_mem_n8(memory, hl.pair, memory.read(pc + 1)); // LD (HL), n8
            case 0x38: return jr_e8(static_cast<int8_t>(memory.read(pc + 1)), get_flag(af.low, FLAG_CARRY)); // JR C, e8
            case 0x39: return add_hl_rr(sp); // ADD HL, SP
            case 0x3A: return ld_a_hlp(memory, false); // LD A, (HL-)
            case 0x3B: return dec_rr(sp); // DEC SP
            case 0x3C: return inc_r(af.high); // INC A
            case 0x3D: return dec_r(af.high); // DEC A
            case 0x3E: return ld_r_n8(af.high, memory.read(pc + 1)); // LD A, n8  
            case 0x40: return ld_r_n8(bc.high, bc.high, 1, 4); // LD B, B
            case 0x41: return ld_r_n8(bc.high, bc.low, 1, 4);  // LD B, C
            case 0x42: return ld_r_n8(bc.high, de.high, 1, 4); // LD B, D
            case 0x43: return ld_r_n8(bc.high, de.low, 1, 4);  // LD B, E
            case 0x44: return ld_r_n8(bc.high, hl.high, 1, 4); // LD B, H
            case 0x45: return ld_r_n8(bc.high, hl.low, 1, 4);  // LD B, L
            case 0x46: return ld_r_n8(bc.high, memory.read(hl.pair), 1, 8); // LD B, (HL)
            case 0x47: return ld_r_n8(bc.high, af.high, 1, 4); // LD B, A
            case 0x48: return ld_r_n8(bc.low, bc.high, 1, 4); // LD C, B
            case 0x49: return ld_r_n8(bc.low, bc.low, 1, 4);  // LD C, C
            case 0x4A: return ld_r_n8(bc.low, de.high, 1, 4); // LD C, D
            case 0x4B: return ld_r_n8(bc.low, de.low, 1, 4);  // LD C, E
            case 0x4C: return ld_r_n8(bc.low, hl.high, 1, 4); // LD C, H
            case 0x4D: return ld_r_n8(bc.low, hl.low, 1, 4);  // LD C, L
            case 0x4E: return ld_r_n8(bc.low, memory.read(hl.pair), 1, 8); // LD C, (HL)
            case 0x4F: return ld_r_n8(bc.low, af.high, 1, 4); // LD C, A
            case 0x50: return ld_r_n8(de.high, bc.high, 1, 4); // LD D, B
            case 0x51: return ld_r_n8(de.high, bc.low, 1, 4);  // LD D, C
            case 0x52: return ld_r_n8(de.high, de.high, 1, 4); // LD D, D
            case 0x53: return ld_r_n8(de.high, de.low, 1, 4);  // LD D, E
            case 0x54: return ld_r_n8(de.high, hl.high, 1, 4); // LD D, H
            case 0x55: return ld_r_n8(de.high, hl.low, 1, 4);  // LD D, L
            case 0x56: return ld_r_n8(de.high, memory.read(hl.pair), 1, 8); // LD D, (HL)
            case 0x57: return ld_r_n8(de.high, af.high, 1, 4); // LD D, A
            case 0x58: return ld_r_n8(de.low, bc.high, 1, 4); // LD E, B
            case 0x59: return ld_r_n8(de.low, bc.low, 1, 4);  // LD E, C
            case 0x5A: return ld_r_n8(de.low, de.high, 1, 4); // LD E, D
            case 0x5B: return ld_r_n8(de.low, de.low, 1, 4);  // LD E, E
            case 0x5C: return ld_r_n8(de.low, hl.high, 1, 4); // LD E, H
            case 0x5D: return ld_r_n8(de.low, hl.low, 1, 4);  // LD E, L
            case 0x5E: return ld_r_n8(de.low, memory.read(hl.pair), 1, 8); // LD E, (HL)
            case 0x5F: return ld_r_n8(de.low, af.high, 1, 4); // LD E, A
            case 0x60: return ld_r_n8(hl.high, bc.high, 1, 4); // LD H, B
            case 0x61: return ld_r_n8(hl.high, bc.low, 1, 4);  // LD H, C
            case 0x62: return ld_r_n8(hl.high, de.high, 1, 4); // LD H, D
            case 0x63: return ld_r_n8(hl.high, de.low, 1, 4);  // LD H, E
            case 0x64: return ld_r_n8(hl.high, hl.high, 1, 4); // LD H, H
            case 0x65: return ld_r_n8(hl.high, hl.low, 1, 4);  // LD H, L
            case 0x66: return ld_r_n8(hl.high, memory.read(hl.pair), 1, 8); // LD H, (HL)
            case 0x67: return ld_r_n8(hl.high, af.high, 1, 4); // LD H, A
            case 0x68: return ld_r_n8(hl.low, bc.high, 1, 4); // LD L, B
            case 0x69: return ld_r_n8(hl.low, bc.low, 1, 4);  // LD L, C
            case 0x6A: return ld_r_n8(hl.low, de.high, 1, 4); // LD L, D
            case 0x6B: return ld_r_n8(hl.low, de.low, 1, 4);  // LD L, E
            case 0x6C: return ld_r_n8(hl.low, hl.high, 1, 4); // LD L, H
            case 0x6D: return ld_r_n8(hl.low, hl.low, 1, 4);  // LD L, L
            case 0x6E: return ld_r_n8(hl.low, memory.read(hl.pair), 1, 8); // LD L, (HL)
            case 0x6F: return ld_r_n8(hl.low, af.high, 1, 4); // LD L, A
            case 0x70: return ld_mem_n8(memory, hl.pair, bc.high); // LD (HL), B
            case 0x71: return ld_mem_n8(memory, hl.pair, bc.low);  // LD (HL), C
            case 0x72: return ld_mem_n8(memory, hl.pair, de.high); // LD (HL), D
            case 0x73: return ld_mem_n8(memory, hl.pair, de.low);  // LD (HL), E
            case 0x74: return ld_mem_n8(memory, hl.pair, hl.high); // LD (HL), H
            case 0x75: return ld_mem_n8(memory, hl.pair, hl.low);  // LD (HL), L
            case 0x77: return ld_mem_n8(memory, hl.pair, af.high); // LD (HL), A
            case 0x78: return ld_r_n8(af.high, bc.high, 1, 4); // LD A, B
            case 0x79: return ld_r_n8(af.high, bc.low, 1, 4);  // LD A, C
            case 0x7A: return ld_r_n8(af.high, de.high, 1, 4); // LD A, D
            case 0x7B: return ld_r_n8(af.high, de.low, 1, 4);  // LD A, E
            case 0x7C: return ld_r_n8(af.high, hl.high, 1, 4); // LD A, H
            case 0x7D: return ld_r_n8(af.high, hl.low, 1, 4);  // LD A, L
            case 0x7E: return ld_r_n8(af.high, memory.read(hl.pair), 1, 8); // LD A, (HL)
            case 0x7F: return ld_r_n8(af.high, af.high, 1, 4); // LD A, A
            case 0x80: return add_a(bc.high); // ADD A, B
            case 0x81: return add_a(bc.low);  // ADD A, C
            case 0x82: return add_a(de.high); // ADD A, D
            case 0x83: return add_a(de.low);  // ADD A, E
            case 0x84: return add_a(hl.high); // ADD A, H
            case 0x85: return add_a(hl.low);  // ADD A, L
            case 0x86: return add_a(memory.read(hl.pair), 1, 8); // ADD A, (HL)
            case 0x87: return add_a(af.high); // ADD A, A
            case 0xA0: return and_a(bc.high); // AND B
            case 0xA1: return and_a(bc.low);  // AND C
            case 0xA2: return and_a(de.high); // AND D
            case 0xA3: return and_a(de.low);  // AND E
            case 0xA4: return and_a(hl.high); // AND H
            case 0xA5: return and_a(hl.low);  // AND L
            case 0xA6: return and_a(memory.read(hl.pair), 1, 8); // AND (HL)
            case 0xA7: return and_a(af.high); // AND A
            case 0xA8: return xor_a(bc.high); // XOR B
            case 0xA9: return xor_a(bc.low);  // XOR C
            case 0xAA: return xor_a(de.high); // XOR D
            case 0xAB: return xor_a(de.low);  // XOR E
            case 0xAC: return xor_a(hl.high); // XOR H
            case 0xAD: return xor_a(hl.low);  // XOR L
            case 0xAE: return xor_a(memory.read(hl.pair), 1, 8); // XOR (HL)
            case 0xAF: return xor_a(af.high); // XOR A
            case 0xB0: return or_a(bc.high); // OR B
            case 0xB1: return or_a(bc.low);  // OR C
            case 0xB2: return or_a(de.high); // OR D
            case 0xB3: return or_a(de.low);  // OR E
            case 0xB4: return or_a(hl.high); // OR H
            case 0xB5: return or_a(hl.low);  // OR L
            case 0xB6: return or_a(memory.read(hl.pair), 1, 8); // OR (HL)
            case 0xB7: return or_a(af.high); // OR A
            case 0xB8: return cp_a(bc.high); // CP B
            case 0xB9: return cp_a(bc.low);  // CP C
            case 0xBA: return cp_a(de.high); // CP D
            case 0xBB: return cp_a(de.low);  // CP E
            case 0xBC: return cp_a(hl.high); // CP H
            case 0xBD: return cp_a(hl.low);  // CP L
            case 0xBE: return cp_a(memory.read(hl.pair), 1, 8); // CP (HL)
            case 0xBF: return cp_a(af.high); // CP A
            case 0xC0: return ret(memory, !get_flag(af.low, FLAG_ZERO), 20); // RET NZ
            case 0xC1: return pop_rr(memory, bc.pair); // POP BC
            case 0xC2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_ZERO)); // JP NZ, a16
            case 0xC3: return jp_a16(memory.read_word(pc + 1)); // JP a16
            case 0xC4: return call_a16(memory, memory.read_word(pc + 1), !get_flag(af.low, FLAG_ZERO)); // CALL NZ, a16
            case 0xC5: return push_rr(memory, bc.pair); // PUSH BC
            case 0xC6: return add_a(memory.read(pc + 1), 2, 8); // ADD A, n8
            case 0xC7: return rst(memory, 0x00); // RST 00H
            case 0xC8: return ret(memory, get_flag(af.low, FLAG_ZERO), 20); // RET Z
            case 0xC9: return ret(memory); // RET
            case 0xCA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_ZERO)); // JP Z, a16
            case 0xCB: return cb_execute_instruction(memory); // CB Prefix
            case 0xCC: return call_a16(memory, memory.read_word(pc + 1), get_flag(af.low, FLAG_ZERO)); // CALL Z, a16
            case 0xCD: return call_a16(memory, memory.read_word(pc + 1)); // CALL a16
            case 0xCF: return rst(memory, 0x08); // RST 08H
            case 0xD0: return ret(memory, !get_flag(af.low, FLAG_CARRY), 20); // RET NC
            case 0xD1: return pop_rr(memory, de.pair); // POP DE
            case 0xD2: return jp_a16(memory.read_word(pc + 1), !get_flag(af.low, FLAG_CARRY)); // JP NC, a16
            case 0xD4: return call_a16(memory, memory.read_word(pc + 1), !get_flag(af.low, FLAG_CARRY)); // CALL NC, a16
            case 0xD5: return push_rr(memory, de.pair); // PUSH DE
            case 0xD7: return rst(memory, 0x10); // RST 10H
            case 0xD8: return ret(memory, get_flag(af.low, FLAG_CARRY), 20); // RET C
            case 0xD9: return ret(memory, true, 16, true); // RETI
            case 0xDA: return jp_a16(memory.read_word(pc + 1), get_flag(af.low, FLAG_CARRY)); // JP C, a16
            case 0xDC: return call_a16(memory, memory.read_word(pc + 1), get_flag(af.low, FLAG_CARRY)); // CALL C, a16
            case 0xDF: return rst(memory, 0x18); // RST 18H
            case 0xE0: return ldh(memory, memory.read(pc + 1), true, 2, 12); // LDH (a8), A
            case 0xE1: return pop_rr(memory, hl.pair); // POP HL
            case 0xE2: return ldh(memory, bc.low, true); // LDH (C), A
            case 0xE5: return push_rr(memory, hl.pair); // PUSH HL
            case 0xE6: return and_a(memory.read(pc + 1), 2, 8); // AND n8
            case 0xE7: return rst(memory, 0x20); // RST 20H
            case 0xE9: return jp_hl(); // JP HL
            case 0xEA: return ld_mem_n8(memory, memory.read_word(pc + 1), af.high, 3, 16); // LD (a16), A
            case 0xEE: return xor_a(memory.read(pc + 1), 2, 8); // XOR n8
            case 0xEF: return rst(memory, 0x28); // RST 28H
            case 0xF0: return ldh(memory, memory.read(pc + 1), false, 2, 12); // LDH A, (a8)
            case 0xF1: return pop_rr(memory, af.pair); // POP AF
            case 0xF2: return ldh(memory, bc.low, false); // LDH A, (C)
            case 0xF3: return di(); // DI
            case 0xF5: return push_rr(memory, af.pair); // PUSH AF
            case 0xF6: return or_a(memory.read(pc + 1), 2, 8); // OR n8
            case 0xF7: return rst(memory, 0x30); // RST 30H
            case 0xFA: return ld_r_n8(af.high, memory.read(memory.read_word(pc + 1)), 3, 16); // LD A, (a16)
            case 0xFB: return ei(); // EI
            case 0xFE: return cp_a(memory.read(pc + 1), 2, 8); // CP n8
            case 0xFF: return rst(memory, 0x38); // RST 38H
            default:
                unimplemented_instruction(opcode, memory.rom);
                return -1; // Indicate error for unimplemented instruction
        }
    }

    int CPU::cb_execute_instruction(Memory& memory) {
        uint8_t opcode = memory.read(pc + 1);
        uint8_t reg_index = opcode & 0x07;  // Extract register bits
        uint8_t* regs[] = {&bc.high, &bc.low, &de.high, &de.low, 
                            &hl.high, &hl.low, nullptr, &af.high};
        // SWAP (0x30-0x37)
        if (opcode >= 0x30 && opcode <= 0x37) {
            if (reg_index == 6) return swap_mem_hl(memory);
            return swap_r(*regs[reg_index]);
        }

        // RES b, r and RES b, (HL) (0x80-0xBF)
        if (opcode >= 0x80 && opcode <= 0xBF) {
            uint8_t bit = (opcode - 0x80) / 8;
            if (reg_index == 6) return res_mem_hl(memory, bit);
            return res(bit, *regs[reg_index]);
        }
        
        unimplemented_instruction(0xCB00 | opcode, memory.rom);
        return -1;
    }
}
