# Game Boy Emulator (gbemu)

A Game Boy emulator implementation in C++ with a complete instruction disassembler.

## Features

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

### Disassemble a ROM

```bash
./build/bin/gbemu rom.gb
```

The disassembler will output a formatted table showing:
- **Addr**: Instruction address (hex)
- **Instruction**: Mnemonic and operands with actual values
- **Flags**: Affected CPU flags (Z, N, H, C) or `-`
- **Len**: Instruction length in bytes
- **Cycles**: CPU cycles (shows conditional variants as fractions, e.g., `8/20`)

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
│   ├── disassembler.cpp       # Instruction disassembler
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

### Implementation Details

- Uses `std::print` for formatted output
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
Addr  Instruction           Flags     Regs   Len  Cycles
----  --------------------  --------  -----  ---  ------
0000  NOP                   -                   1  4    
0001  LD $1234              -                   3  12   
0004  LD B, B               -                   1  4    
0005  ADD A, B              Z,0,H,C             1  4    
0079  RLC B                 Z,0,0,C             2  8    
00F9  BIT 0, B              Z,0,1,-             2  8    
0179  RES 0, B              -                   2  8    
01F9  SET 0, B              -                   2  8    
```

## Development

### Design Approach

- Struct-based instruction representation
- Direct binary file I/O with `fopen`/`fread`
- RAII for resource management

## Future Work

- CPU emulation core
- Memory management (RAM, VRAM, cartridge)
- Interrupt handling
- Graphics/PPU emulation
- Sound/APU emulation
- Debugger interface

## License

MIT

## Resources

- [Game Boy CPU Manual](https://gbdev.io/pandocs/)
