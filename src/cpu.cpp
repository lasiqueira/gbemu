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
    
    // Execute one instruction, return cycles taken
    int CPU::execute_instruction(Memory& memory) {
        uint8_t opcode = memory.read(pc);
        switch (opcode) {
            case 0x00: // NOP
                return nop();
            default:
                unimplemented_instruction(opcode, memory.rom);
                return -1; // Indicate error for unimplemented instruction
        }
    }
}
