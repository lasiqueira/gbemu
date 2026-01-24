# Game Boy Emulator (gbemu)

A Game Boy emulator implementation in C++ with a complete instruction disassembler.

## Features

- **CPU Emulation Core**
  - Accurate Game Boy CPU (Sharp LR35902) emulation
  - Complete register set (AF, BC, DE, HL, SP, PC)
  - Flag register management (Z, N, H, C)
  - Instruction execution with cycle-accurate timing
  - Frame-based execution loop (~59.7 Hz)

- **Memory Management**
  - 64KB address space
  - ROM loading support
  - Cartridge memory mapping (0x0000-0x7FFF)
  - Work RAM (0xC000-0xDFFF)
  - High RAM (0xFF80-0xFFFE)

- **Complete Game Boy Instruction Disassembler**
  - All 256 standard opcodes (0x00-0xFF)
  - All 256 CB-prefixed opcodes (0xCB00-0xCBFF)
  - Comprehensive instruction metadata (flags, cycle counts, lengths)
  - Formatted output with operand values

## Building

### Prerequisites
- C++23 compatible compiler (Clang 17+, GCC 13+)
- CMake 3.20+

### Build Steps

```bash
# Configure build (Release by default)
cmake -B build

# Build project
cmake --build build

# Optional: Run tests
ctest --test-dir build --verbose
```

### Build Options

**Build with custom optimization:**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

**Windows: Use clang-cl for better optimization:**
```powershell
cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 && cmake -B build -G "Visual Studio 17 2022" -T ClangCL'
cmake --build build --config Release
```

The build system uses:
- `-march=native` on Linux/macOS/Windows (clang-cl) for maximum optimization
- `/arch:AVX2` on Windows (MSVC only)
- `-O3` optimization level

## Usage

### Run Emulator

```bash
# Linux/macOS
./build/bin/gbemu rom.gb

# Windows
.\build\bin\Release\gbemu.exe rom.gb

# Disassemble only (no execution)
./build/bin/gbemu rom.gb --disassemble       # Linux/macOS
.\build\bin\Release\gbemu.exe rom.gb -d      # Windows (short form)
```

**Disassembler Mode** (`--disassemble` or `-d`): Outputs a formatted table showing:
- **Addr**: Instruction address (hex)
- **Instruction**: Mnemonic and operands with actual values
- **Flags**: Affected CPU flags (Z, N, H, C) or `-`
- **Len**: Instruction length in bytes
- **Cycles**: CPU cycles (shows conditional variants as fractions, e.g., `8/20`)

### Debug Mode

To enable debug tracing (shows each instruction as executed), build with debug features:

```bash
# Configure with debug features enabled
cmake -B build -DGBEMU_DEBUG=ON

# Build
cmake --build build --config Release  # Windows
cmake --build build                   # Linux/macOS

# Run - will show instruction trace
./build/bin/gbemu rom.gb                     # Linux/macOS
.\build\bin\Release\gbemu.exe rom.gb         # Windows
```

**Debug output** shows detailed CPU state and instruction trace:
```
Executing: 0xC3 at PC: 0x0101
JP $0150
CPU State after execution:
A: 00  B: 00  C: 00  D: 00  E: 00  H: 00  L: 00  SP: FFFE  PC: 0150
Flags: Z=0 N=0 H=0 C=0
Cycles: 16
---
```

### Run Tests

```bash
# Linux/macOS
ctest --test-dir build --verbose

# Windows (with Visual Studio generator)
ctest --test-dir build -C Release --verbose

# Or run test binary directly
./build/bin/test_disasm              # Linux/macOS
.\build\bin\Release\test_disasm.exe  # Windows
```

The test suite covers all 256+ Game Boy instructions with complete register and bit position coverage.

## Project Structure

```
.
├── CMakeLists.txt              # Build configuration
├── .gitignore                  # Git ignore rules
├── src/
│   ├── main.cpp               # Entry point (ROM loading)
│   ├── gameboy.h              # Game Boy system declarations
│   ├── gameboy.cpp            # Game Boy system implementation
│   ├── cpu.h                  # CPU declarations
│   ├── cpu.cpp                # CPU implementation
│   ├── memory.h               # Memory declarations
│   ├── memory.cpp             # Memory implementation
│   ├── disassembler.h         # Instruction disassembler declarations
│   ├── disassembler.cpp       # Instruction disassembler implementation
│   └── test_disasm.cpp        # Comprehensive test suite
└── README.md                   # This file
```

## Architecture

### Disassembler

The disassembler is built around a struct-based instruction representation:

```cpp
struct Instruction {
    std::string mnemonic;
    std::string operands;
    std::vector<std::string> affected_flags;
    uint8_t length;
    uint8_t cycles;
    uint8_t cycles_taken;  // For conditional instructions
};
```

**Key Components:**
- `decode_instruction()`: Decodes standard opcodes (0x00-0xFF)
- `decode_cb_instruction()`: Decodes CB-prefixed opcodes (0xCB00-0xCBFF)
- `Instruction::format()`: Formats instruction with actual operand values
- `print_disassembly()`: Batch disassembly with formatted output

### CPU Emulation

The CPU emulation uses a struct-based design with union register pairs for efficient 8-bit/16-bit access:

```cpp
struct CPU {
    RegisterPair bc, de, hl, af;  // 8/16-bit register pairs
    uint16_t sp, pc;               // Stack pointer, Program counter
    
    int execute(Memory& memory);   // Execute one instruction
};
```

**Key Features:**
- Register pairs use unions for dual 8-bit/16-bit access (e.g., `bc.high`, `bc.low`, `bc.pair`)
- Flag register helpers for Z, N, H, C flags
- Cycle-accurate instruction timing
- Integration with disassembler for debugging unimplemented opcodes

### Memory System

The memory system provides a flat 64KB address space with proper region mapping:

```cpp
struct Memory {
    std::vector<uint8_t> rom;      // Cartridge ROM (0x0000-0x7FFF)
    std::array<uint8_t, 8192> wram; // Work RAM (0xC000-0xDFFF)
    std::array<uint8_t, 127> hram;  // High RAM (0xFF80-0xFFFE)
    
    uint8_t read(uint16_t address);
    void write(uint16_t address, uint8_t value);
};
```

### Game Boy System

The `GameBoy` struct integrates CPU and memory with timing control:

```cpp
struct GameBoy {
    Memory memory;
    cpu::CPU cpu;
    
    int step();                    // Execute one instruction
    int step_frame();              // Execute one frame (~70224 cycles)
    void run();                    // Main emulation loop
};
```

**Timing Constants:**
- CPU Frequency: 4.194304 MHz
- Frame Rate: ~59.7 Hz
- Cycles per Frame: ~70224

### Implementation Details

- Uses C++23 features including `std::print` and `std::format` for formatted output
- Modular design with separate header and implementation files
- Cross-platform file handling with `std::filesystem`
- Optimized builds with `-march=native` (Linux/macOS/Windows with clang-cl) or `/arch:AVX2` (Windows with MSVC)

## Testing

The test suite (`test_disasm.cpp`) includes:
- All 256 standard Game Boy opcodes
- All 256 CB-prefixed opcodes
- Complete coverage of:
  - All 8 registers (B, C, D, E, H, L, (HL), A)
  - All 8 bit positions (0-7) for BIT/RES/SET instructions
  - Rotate, shift, and bitwise operations
  - Load, arithmetic, and control flow instructions

Total test coverage: 630+ bytes covering 350+ unique instructions

## Example Output

```
Addr  Instruction           Flags       Len  Cycles
----  --------------------  --------    ---  ------
0000  NOP                   -             1  4    
0001  LD $1234              -             3  12   
0004  LD B, B               -             1  4    
0005  ADD A, B              Z,0,H,C       1  4    
0079  RLC B                 Z,0,0,C       2  8    
00F9  BIT 0, B              Z,0,1,-       2  8    
0179  RES 0, B              -             2  8    
01F9  SET 0, B              -             2  8    
```

## Development

### Design Approach

- Struct-based instruction representation
- Direct binary file I/O with `fopen`/`fread`
- RAII for resource management

## Current Status

**CPU Instructions: ~190/256 implemented (~74%)**

Implemented instruction families:
- **Control flow**: JP, JR, CALL, RET (conditional/unconditional)
- **Data transfer**: LD (all variants: r8, r16, immediate, (HL+/-), direct memory, (BC)/(DE))
- **Arithmetic**: INC, DEC, ADD, SUB, CP, ADC, SBC (8-bit and 16-bit variants)
- **Logical**: XOR, OR, AND, BIT, RES, SET, RL, RLC, RR, RRC, SLA, SRA, SRL, SWAP
- **Interrupt control**: DI, EI, RETI
- **Stack operations**: PUSH, POP (all register pairs)
- **Miscellaneous**: NOP, HALT, DAA, CPL, CCF, SCF

**Milestone**: Successfully boots Tetris ROM to title screen

- ✅ Boots and initializes Tetris ROM completely
- ✅ LCD/PPU timing system working (LY register, 154 scanlines, VBlank interrupts)
- ✅ VBlank interrupt handling functional
- ✅ Main game loop executes at ~60 FPS
- ✅ Joypad register reads working (game waiting for input at title screen)
- ⚠️ Additional unimplemented instructions likely exist but haven't been encountered yet

## Future Work

- **Input handling** (joypad input implementation - next priority)
- **Graphics/PPU emulation** (tiles, sprites, background rendering)
- Remaining CPU instructions as discovered during gameplay
- Sound/APU emulation (4 audio channels)
- Debugger interface
- Cartridge types (MBC1, MBC3, MBC5)
- Save state functionality

## License

MIT

## Resources

- [Game Boy CPU Manual](https://gbdev.io/pandocs/)
