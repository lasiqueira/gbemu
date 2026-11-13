# Game Boy Emulator (gbemu)

A Game Boy and Game Boy Color emulator implementation in C++ with complete graphics, audio, input, and comprehensive documentation.

## Documentation

Detailed development documentation with step-by-step explanations is available in the `docs/` folder. Open `docs/index.html` in a browser to view:

- Disassembler implementation
- CPU and memory systems
- Instruction set implementation (Part 1, Part 2 and completing the CPU)
- Input system and PPU rendering
- Memory bank controllers and battery saves
- Audio processing unit (APU)
- Game Boy Color support

## Screenshot

![Tetris Title Screen](docs/images/tetris_title_debug.png)

*Tetris running in the emulator with debug panels showing CPU state and registers*

## Features

- **CPU Emulation Core**
  - Accurate Game Boy CPU (Sharp LR35902) emulation
  - Complete register set (AF, BC, DE, HL, SP, PC)
  - Flag register management (Z, N, H, C)
  - Instruction execution with cycle-accurate timing
  - Frame-based execution loop (~59.7 Hz)
  - Interrupt handling (VBlank, LCD STAT, Timer, Serial, Joypad)
  - HALT bug (IME=0 with pending interrupt causes PC not to advance on next fetch)

- **PPU (Picture Processing Unit)**
  - Background layer with scrolling (SCX/SCY registers)
  - Window layer with line counter
  - Sprite/Object rendering (8x8 and 8x16 modes)
  - OAM scanning with 10 sprites per scanline limit
  - Sprite priority system and transparency
  - X/Y flipping for sprites
  - All PPU modes and timing (OAM Search, Drawing, H-Blank, V-Blank)
  - Frames are completed at V-Blank, so the displayed image never tears
  - LCDC and STAT register support

- **Input System**
  - Joypad register (0xFF00) with button group selection
  - Keyboard input support (Z/X for A/B, arrows, Enter/Shift)
  - Gamepad/Controller support via SDL3
    - Xbox controllers (Xbox One, Series X|S)
    - PlayStation controllers (DualShock 4, DualSense)
    - Nintendo Pro Controllers
    - Generic controllers via SDL gamepad database
  - Hot-plugging support for controllers

- **Timer System**
  - DIV register (0xFF04)
  - TIMA/TMA/TAC with all 4 clock speeds
  - Timer overflow interrupt

- **Serial Port**
  - Master-mode (0x81) transfers complete immediately for Blargg test compatibility
  - Slave-mode (0x80) transfers stall — prevents false link-cable detection

- **Memory Management**
  - 64KB address space
  - ROM loading support
  - Cartridge memory mapping (0x0000-0x7FFF)
  - Video RAM (VRAM) at 0x8000-0x9FFF
  - OAM (Object Attribute Memory) at 0xFE00-0xFE9F
  - Work RAM (0xC000-0xDFFF)
  - I/O registers (0xFF00-0xFF7F)
  - High RAM (0xFF80-0xFFFE)
  - Memory Bank Controllers (MBC1, MBC2, MBC3, MBC5)
    - ROM banking (up to 512 banks / 8MB)
    - External RAM banking with enable/disable
    - MBC1 mode 0/1 (ROM/RAM banking modes)
    - MBC2 built-in 512×4-bit RAM
    - MBC3 RTC register latch
    - MBC5 9-bit ROM bank number

- **Audio Processing Unit (APU)**
  - CH1: Square wave with frequency sweep (NR10–NR14)
  - CH2: Square wave (NR21–NR24)
  - CH3: Wave channel with 32-nibble custom waveform (NR30–NR34)
  - CH4: Noise channel with 15-bit/7-bit LFSR (NR41–NR44)
  - Frame sequencer at 512 Hz: length (256 Hz), sweep (128 Hz), envelope (64 Hz)
  - Volume envelope on CH1, CH2, CH4 (configurable pace, fade in/out)
  - Frequency sweep on CH1: overflow check on trigger and on each clock; negate-mode exit immediately disables channel
  - NR51 per-channel stereo panning, NR50 master volume (1–8 per side)
  - NR52 master enable/disable; length counters preserved through power cycles (DMG hardware behaviour)
  - NR41 (CH4 length register) writable while APU powered off (DMG hardware behaviour)
  - Wave channel read/write while active only succeeds on exact-cycle coincidence with CH3's own sample fetch, else read returns 0xFF / write is dropped (DMG hardware behaviour)
  - SDL3 stereo audio stream at 44,100 Hz, 16-bit signed
  - Frame timing paced on the audio queue, so the sound card never underruns

- **Game Boy Color**
  - Hardware model selection: `--model=cgb|dmg|auto`, with CGB as the default
  - Switchable at runtime with hotkeys (pause, reset, cycle model, toggle colour correction)
  - Two 8 KiB VRAM banks (VBK) and eight 4 KiB WRAM banks (SVBK)
  - BG and OBJ colour palette RAM (BGPI/BGPD/OBPI/OBPD) with auto-increment and optional LCD colour correction
  - BG/window tile attributes (palette, tile bank, X/Y flip, BG-to-OBJ priority) and OBJ attributes with the OPRI priority mode
  - Double-speed mode (KEY1 + STOP), with the PPU, APU and RTC kept on real time
  - HDMA and GDMA VRAM transfers (HBlank and general-purpose), paused during HALT
  - DMG games on CGB hardware run in compatibility mode with the colours the real boot ROM picks from the game's title checksum
  - DMG-only quirks (OAM bug, wave RAM lockout and retrigger corruption, length counters surviving power-off) are switched off on CGB hardware, and CGB-only registers are hidden from DMG cartridges
  - See [docs/colour.html](docs/colour.html)

- **Complete Game Boy Instruction Disassembler**
  - All 256 standard opcodes (0x00-0xFF)
  - All 256 CB-prefixed opcodes (0xCB00-0xCBFF)
  - Comprehensive instruction metadata (flags, cycle counts, lengths)
  - Formatted output with operand values

## Building

### Prerequisites
- C++23 compatible compiler (Clang 17+, GCC 13+)
- CMake 3.20+
- Git (for submodule initialization)

### Getting the Source

After cloning the repository, initialize and fetch the submodules (SDL3 and ImGui):

```bash
# Clone and initialize submodules in one step
git clone --recurse-submodules https://github.com/yourusername/gbemu.git

# Or, if you already cloned without submodules
git submodule update --init --recursive
```

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
# If you previously configured with MSVC, remove the build dir first
Remove-Item -Recurse -Force build
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

# Choose the hardware model (default: cgb)
.\build\bin\Release\gbemu.exe rom.gb --model=cgb    # Game Boy Color (DMG games run in colour compat mode)
.\build\bin\Release\gbemu.exe rom.gb --model=dmg    # original Game Boy
.\build\bin\Release\gbemu.exe rom.gb --model=auto   # CGB cartridge -> CGB, otherwise DMG
```

With the default (`cgb`), DMG games run in colour, with the palette the real Game Boy Color boot ROM would pick for the title, and without the DMG-only hardware quirks (such as the OAM bug). Use `--model=dmg` for original-hardware behaviour, for example when running the Blargg DMG test ROMs.

**Hotkeys:**
- **F1** - Pause / resume
- **F2** - Reset (reloads the ROM; battery saves are written first)
- **F3** - Cycle the model `auto → DMG → CGB`, then reload
- **F4** - Toggle CGB colour correction (kept across reset and model changes)

The window title shows the selected model and what is actually running, for example `[auto -> DMG]` or `[CGB -> CGB (DMG compat)]`. Forcing DMG on a CGB-only cartridge runs it as a DMG game and will probably not display correctly.

To run the Blargg DMG test ROMs headlessly (no window), pass `--headless`, and use `--model=dmg` to match original-hardware results:

```bash
.\build\bin\Release\gbemu.exe gb-test-roms-master\cpu_instrs\cpu_instrs.gb --headless --model=dmg
```

**Controls:**

Keyboard:
- **Z** - A button
- **X** - B button
- **Arrow Keys** - D-Pad
- **Enter** - Start
- **Shift** - Select

Gamepad (Xbox/PlayStation/Nintendo Pro):
- **South button** (A/✕/B) - A button
- **East button** (B/○/A) - B button
- **D-Pad** - D-Pad
- **Start/Menu/+** - Start
- **Back/Share/−** - Select

Controllers are automatically detected and can be hot-plugged during gameplay.

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

**Debug output** shows detailed CPU state and instruction trace. Debug builds also show the ImGui panels (disassembly, CPU state, memory viewer), which include a colour-correction checkbox and a model selector:
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
│   ├── main.cpp               # Entry point: SDL window, audio, input, hotkeys, ImGui debug panels
│   ├── constants.h            # Register addresses, memory map, header offsets and bit layouts
│   ├── gameboy.h              # Game Boy system declarations
│   ├── gameboy.cpp            # System loop, model selection, post-boot state, speed switch, HDMA
│   ├── cpu.h                  # CPU declarations
│   ├── cpu.cpp                # CPU implementation
│   ├── memory.h               # Memory system declarations
│   ├── memory.cpp             # Memory map, MBCs, banking, I/O registers, battery saves
│   ├── ppu.h                  # PPU/graphics declarations
│   ├── ppu.cpp                # PPU/graphics implementation (DMG and CGB rendering)
│   ├── apu.h                  # Audio processing unit declarations
│   ├── apu.cpp                # Four-channel APU and mixer
│   ├── compat_palettes.h      # DMG-on-CGB palette lookup interface
│   ├── compat_palettes.cpp    # Title checksum lookup, as done by the CGB boot ROM
│   ├── compat_palette_data.h  # Boot ROM checksum and palette tables
│   ├── disassembler.h         # Instruction disassembler declarations
│   ├── disassembler.cpp       # Instruction disassembler implementation
│   └── test_disasm.cpp        # Comprehensive test suite
├── docs/                       # HTML documentation
│   ├── index.html             # Introduction and table of contents
│   ├── disassembler.html      # Disassembler documentation
│   ├── cpu_and_memory.html    # CPU and memory systems
│   ├── instructions_part1.html # Instruction implementation
│   ├── instructions_part2.html # More instructions
│   ├── instructions_final.html # Completing the instruction set
│   ├── input_and_rendering.html # Input and PPU systems
│   ├── memory_bank_controller.html # MBC1/2/3/5 and battery saves
│   ├── apu.html               # Audio processing unit
│   ├── colour.html            # Game Boy Color support
│   ├── images/                # Screenshots
│   └── style.css              # Documentation styling
├── vendor/                     # Third-party dependencies (SDL3, ImGui)
└── README.md                   # This file
```

Test ROMs (for example the Blargg suites in `gb-test-roms-master/`) are not tracked in the repository.

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
    
    int execute_instruction(Memory& memory);  // Execute one instruction, returns cycles
};
```

**Key Features:**
- Register pairs use unions for dual 8-bit/16-bit access (e.g., `bc.high`, `bc.low`, `bc.pair`)
- Flag register helpers for Z, N, H, C flags
- M-cycle-accurate bus timing: every memory access ticks the PPU, APU and timers at the moment it happens
- Integration with disassembler for debugging unimplemented opcodes

### Memory System

The memory system provides a 64KB address space with banked regions for the Game Boy Color:

```cpp
struct Memory {
    std::vector<uint8_t> rom;        // Cartridge ROM, banked by the MBC
    std::vector<uint8_t> ext_ram;    // Cartridge RAM (battery-backed when present)
    uint8_t vram[0x4000];            // Two 8 KiB VRAM banks (second bank is CGB only)
    uint8_t wram[0x8000];            // Eight 4 KiB WRAM banks (CGB); two on DMG
    uint8_t bg_palette_ram[64];      // CGB colour RAM
    uint8_t obj_palette_ram[64];
    
    uint8_t read(uint16_t address) const;
    void write(uint16_t address, uint8_t value);
};
```

I/O register reads and writes are model-aware: CGB-only registers (KEY1, VBK, HDMA, SVBK) are hidden from DMG cartridges, and the APU registers are forwarded to the APU.

### PPU (Picture Processing Unit)

The PPU handles all graphics rendering with three layers and renders each scanline to an RGBA framebuffer:

```cpp
struct PPU {
    std::array<uint32_t, 160 * 144> rgba_buffer;  // RGBA framebuffer
    std::array<Sprite, 10> visible_sprites;       // Up to 10 sprites per scanline
    std::array<uint32_t, 32> bg_palette_rgba;     // CGB palettes, converted to RGBA once
    std::array<uint32_t, 32> obj_palette_rgba;
    uint8_t window_line_counter;                  // Internal window line counter
    
    void step(int cycles, Memory& memory);       // Execute PPU for N cycles
    void render_scanline(Memory& memory);        // Render current scanline
};
```

**Key Features:**
- Background rendering with tile data from VRAM
- Window layer for UI overlays (separate from background scrolling)
- Sprite rendering with OAM scanning
- 4 PPU modes: OAM Search (80 cycles), Drawing (172 cycles), H-Blank (204 cycles), V-Blank
- DMG sprite priority (lower X wins) or CGB OAM-order priority, selected by OPRI
- Sprite attributes: X/Y flip, palette selection, BG priority
- CGB colour: per-tile palettes and banks, flips, BG-to-OBJ priority, optional LCD colour correction
- DMG games in compatibility mode index into CGB palette RAM through BGP/OBP0/OBP1

### Audio (APU)

The APU runs four channels (two square waves, a wave channel and a noise channel) clocked by a 512 Hz frame sequencer. `APU::step` advances the channels and mixes a stereo sample at 44.1 kHz into a buffer that `main.cpp` hands to an SDL3 audio stream once per frame. Some behaviours depend on the hardware model, such as wave RAM access while a channel plays and length counters across power-off, so the APU keeps its own copy of the `cgb_mode` flag.

### Game Boy System

The `GameBoy` struct integrates CPU, memory, PPU and APU with timing control:

```cpp
struct GameBoy {
    Memory memory;
    CPU cpu;
    PPU ppu;
    APU apu;
    
    void load_rom(const std::vector<uint8_t>& rom, const std::string& rom_path);
    int step();                    // Execute one instruction
    int step_frame();              // Run until the PPU enters V-Blank
};
```

`load_rom` resets the machine, loads the cartridge, resolves the hardware model (`DMG`, `CGB`, or `auto` from the cartridge header), applies the post-boot register state for that model, and restores any battery save. Resetting the emulator means calling it again.

**Timing Constants:**
- CPU Frequency: 4.194304 MHz
- Frame Rate: ~59.7 Hz
- Cycles per Frame: ~70224
- Cycles per Scanline: 456
- Scanlines per Frame: 154 (144 visible + 10 V-Blank)
- In CGB double-speed mode the CPU and timers run twice as fast while the PPU, APU and RTC stay on real time

### Implementation Details

- Uses C++23 features including `std::print` and `std::format` for formatted output
- SDL3 for window management, rendering, and input handling
- ImGui for debug UI and CPU state visualization
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

**Fully playable!**

The emulator successfully runs Tetris and Pokémon in both original Game Boy and Game Boy Color modes, passes all 11 Blargg cpu_instrs tests, and produces full stereo audio. MBC1–MBC5 cartridge support enables the full commercial game library, and DMG games run in colour by default.

### Implemented Features

**CPU Instructions: ~100% of instruction set**
- ✅ Control flow: JP, JR, CALL, RET (conditional/unconditional)
- ✅ Data transfer: LD (all variants: r8, r16, immediate, (HL+/-), direct memory, (BC)/(DE))
- ✅ Arithmetic: INC, DEC, ADD, SUB, CP, ADC, SBC (8-bit and 16-bit variants, all registers + immediate)
- ✅ Logical: XOR, OR, AND, BIT, RES, SET, RL, RLC, RR, RRC, SLA, SRA, SRL, SWAP
- ✅ Interrupt control: DI, EI, RETI
- ✅ Stack operations: PUSH, POP (all register pairs)
- ✅ Miscellaneous: NOP, HALT (with HALT bug), DAA, CPL, CCF, SCF, RLCA, RRCA, RLA, RRA

**Graphics/PPU:**
- ✅ Background rendering with scrolling (SCX/SCY)
- ✅ Window layer with proper line counter
- ✅ Sprite rendering (8x8 and 8x16 modes)
- ✅ OAM scanning (40 sprites, 10 per scanline)
- ✅ Sprite attributes (priority, flip X/Y, palettes)
- ✅ All PPU modes and timing (456 cycles per scanline)
- ✅ V-Blank, LCD STAT, and Timer interrupts

**Timers:**
- ✅ DIV register (0xFF04)
- ✅ TIMA/TMA/TAC timer with all 4 clock speeds
- ✅ Timer overflow interrupt

**Serial:**
- ✅ Master-mode (internal clock) transfers complete immediately
- ✅ Slave-mode (external clock) transfers stall — no false link-cable detection

**Input:**
- ✅ Joypad register (0xFF00) implementation
  - ✅ Joypad interrupt (INT_JOYPAD) on button press
  - ✅ Wake from STOP on joypad interrupt
- ✅ Keyboard control (Z/X/Arrows/Enter/Shift)
- ✅ Gamepad support (Xbox/PlayStation/Nintendo Pro)
- ✅ Controller hot-plugging

**Memory Bank Controllers:**
- ✅ MBC1 with mode 0/1 (ROM/RAM banking)
- ✅ MBC2 with built-in 512×4-bit RAM
- ✅ MBC3 with RTC register latch and clock ticking
- ✅ MBC5 with 9-bit ROM bank number
- ✅ External RAM enable/disable
- ✅ Cartridge header parsing (type, ROM size, RAM size)
- ✅ Battery-backed save files (.sav) with RTC persistence

**Audio (APU):**
- ✅ CH1 square wave: all duty cycles, envelope, frequency sweep
- ✅ CH2 square wave: all duty cycles, envelope
- ✅ CH3 wave channel: 32-nibble wave RAM, four output levels
- ✅ CH4 noise channel: 15-bit and 7-bit LFSR modes
- ✅ Frame sequencer (length, sweep, envelope clocks)
- ✅ Volume envelopes on CH1, CH2, CH4
- ✅ Frequency sweep on CH1: overflow on trigger and clock, negate-mode exit disables channel
- ✅ NR51 stereo panning, NR50 master volume, NR52 master enable with DMG power-cycle behaviour
- ✅ Wave channel active read/write only succeeds on exact-cycle coincidence with CH3's own fetch
- ✅ 44,100 Hz stereo output via SDL3 audio stream
- ✅ Frame pacing driven by the audio queue (no underruns or pops)

**Game Boy Color:**
- ✅ Hardware model resolved at load time (`cgb` / `dmg` / `auto`), CGB by default
- ✅ CGB post-boot state (A = $11) and per-model reset
- ✅ VRAM banking (VBK) and WRAM banking (SVBK)
- ✅ BG and OBJ palette RAM with auto-increment, RGB555 conversion and optional colour correction
- ✅ BG/window tile attributes (palette, bank, X/Y flip, priority) and OBJ attributes with OPRI priority mode
- ✅ BG-to-OBJ priority resolution
- ✅ Double-speed mode (KEY1 + STOP)
- ✅ HDMA / GDMA, including the HALT pause
- ✅ DMG-on-CGB compatibility mode with authentic per-game palettes from the boot ROM's title checksum table, and the compat-mode boot register values
- ✅ DMG-only quirks gated off on CGB hardware; CGB-only registers hidden from DMG cartridges
- ✅ CGB wave RAM power-on pattern
- ✅ Hotkeys for pause, reset, model cycling and colour correction

**Documentation:**
- ✅ Comprehensive HTML documentation covering all implemented systems
- ✅ Step-by-step development log

### Test ROM Results

The results below are for the original Game Boy (`--model=dmg`). Game Boy Color results are listed at the end of this section.

**Blargg cpu_instrs — all tests pass:**
- ✅ 01: special
- ✅ 02: interrupts
- ✅ 03: op sp,hl
- ✅ 04: op r,imm
- ✅ 05: op rp
- ✅ 06: ld r,r
- ✅ 07: jr,jp,call,ret,rst
- ✅ 08: misc instrs
- ✅ 09: op r,r
- ✅ 10: bit ops
- ✅ 11: op a,(hl)

**Blargg dmg_sound — 11 of 12 tests pass:**
- ✅ 01: registers
- ✅ 02: len ctr
- ✅ 03: trigger
- ✅ 04: sweep
- ✅ 05: sweep details
- ✅ 06: overflow on trigger
- ✅ 07: len sweep period sync
- ✅ 08: len ctr during power
- ✅ 09: wave read while on
- ❌ 10: wave trigger while on (APU wave-channel retrigger quirk not fully matching hardware)
- ✅ 11: regs after power
- ✅ 12: wave write while on

**Blargg instr_timing — passes:**
- ✅ instr_timing

**Blargg mem_timing — all tests pass:**
- ✅ 01: read_timing
- ✅ 02: write_timing
- ✅ 03: modify_timing

**Blargg mem_timing-2 — all tests pass:**
- ✅ 01: read_timing
- ✅ 02: write_timing
- ✅ 03: modify_timing

**Blargg oam_bug — 7 of 8 tests pass standalone (8/8 in the combined ROM):**
- ✅ 1-lcd_sync
- ✅ 2-causes
- ✅ 3-non_causes
- ✅ 4-scanline_timing
- ✅ 5-timing_bug
- ✅ 6-timing_no_bug
- ⚠️ 7-timing_effect (passes as test 07 inside the combined ROM; standalone build's console printing exceeds the 120s headless budget)
- ✅ 8-instr_effect

**Blargg halt_bug — visual-only test (no serial/RAM output, not measurable headlessly)**

**Blargg interrupt_time — not measurable headlessly (manual-comparison test, no pass/fail signature)**

**Game Boy Color (`--model=cgb`):**
- ✅ Blargg cgb_sound — all 12 tests pass
- ✅ cpu_instrs, instr_timing, mem_timing and mem_timing-2 pass
- ➖ oam_bug and dmg_sound fail by design: they test DMG-only quirks (the OAM bug, wave RAM timing) that do not exist on CGB hardware
- ⏳ Visual CGB test ROMs ([cgb-acid2](https://github.com/mattcurrie/cgb-acid2), [MagenTests](https://github.com/alloncm/MagenTests)) and double speed with a ROM that actually switches have not been run

See [docs/blargg_test_status.md](docs/blargg_test_status.md) for the full pass/fail matrix,
per-opcode failure detail, and a prioritized root-cause/remediation backlog for the remaining failures.

**Tetris — fully playable:**
- ✅ Boots to title screen with full graphics
- ✅ Title screen demo timeout (timer-driven countdown)
- ✅ 1-player / 2-player mode selection
- ✅ Falling pieces with correct sprite rendering
- ✅ Background and window layer for playfield and UI
- ✅ Input functional (keyboard and gamepad)
- ✅ V-Blank, LCD STAT, and timer interrupts working
- ✅ Serial port: master-mode transfers complete; slave-mode stalls (no false link-cable detection)

**Pokémon Blue — fully playable:**
- ✅ Title screen, intro sequence, main menu and save/load
- ✅ Overworld exploration with full background music (CH1–CH3) and sound effects (CH4)
- ✅ Battle system with CH3 wave melody and CH4 noise effects
- ✅ APU power cycling between screens (buildings, battles, menu transitions)
- ✅ Runs in colour on CGB hardware, using the palette the real boot ROM picks for the game

## Future Work

- **Frontend UI** (ROM picker, display and audio settings, controller remapping), so games no longer have to be started from the command line
- **Save state functionality**
- **Debugger interface** (memory viewer, breakpoints)
- **Remaining Game Boy Color validation**: visual test ROMs and speed-switching games

## License

MIT

## Resources

- [Game Boy CPU Manual](https://gbdev.io/pandocs/)
