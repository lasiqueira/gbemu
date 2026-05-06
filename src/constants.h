#pragma once
#include <cstdint>

// Game Boy timing constants
constexpr int CPU_FREQUENCY = 4194304;        // 4.194304 MHz (cycles per second)
constexpr double FRAME_RATE = 59.7;           // ~59.7 Hz (frames per second)
constexpr int CYCLES_PER_FRAME = static_cast<int>(CPU_FREQUENCY / FRAME_RATE);  // ~70224 cycles

// LCD timing constants
constexpr int CYCLES_PER_SCANLINE = 456; // 456 cycles per scanline
constexpr int SCANLINES_PER_FRAME = 154; // 154 scanlines per frame
constexpr int VBLANK_SCANLINE = 144; // V-Blank starts at scanline 144

// I/O Register Addresses
constexpr uint16_t IO_JOYPAD = 0xFF00; // Joypad
constexpr uint16_t IO_SB     = 0xFF01; // Serial Transfer Data
constexpr uint16_t IO_SC     = 0xFF02; // Serial Transfer Control
constexpr uint16_t IO_DIV    = 0xFF04; // Divider Register
constexpr uint16_t IO_TIMA   = 0xFF05; // Timer Counter
constexpr uint16_t IO_TMA    = 0xFF06; // Timer Modulo
constexpr uint16_t IO_TAC    = 0xFF07; // Timer Control
constexpr uint16_t IO_IF     = 0xFF0F; // Interrupt Flag
constexpr uint16_t IO_LCDC   = 0xFF40; // LCD Control
constexpr uint16_t IO_STAT   = 0xFF41; // LCD Status
constexpr uint16_t IO_SCY    = 0xFF42; // Scroll Y
constexpr uint16_t IO_SCX    = 0xFF43; // Scroll X
constexpr uint16_t IO_LY     = 0xFF44; // LCD Y-Coordinate
constexpr uint16_t IO_LYC    = 0xFF45; // LY Compare
constexpr uint16_t IO_DMA    = 0xFF46; // DMA Transfer
constexpr uint16_t IO_BGP    = 0xFF47; // BG Palette Data
constexpr uint16_t IO_OBP0   = 0xFF48; // Object Palette 0
constexpr uint16_t IO_OBP1   = 0xFF49; // Object Palette 1
constexpr uint16_t IO_WY     = 0xFF4A; // Window Y Position
constexpr uint16_t IO_WX     = 0xFF4B; // Window X Position
constexpr uint16_t IO_IE     = 0xFFFF; // Interrupt Enable

// Memory map region boundaries
constexpr uint16_t ADDR_ROM_BANK0_END  = 0x4000; // End of fixed ROM bank 0
constexpr uint16_t ADDR_VRAM_START     = 0x8000; // Start of VRAM / end of switchable ROM
constexpr uint16_t ADDR_EXT_RAM_START  = 0xA000; // Start of external cartridge RAM
constexpr uint16_t ADDR_WRAM_START     = 0xC000; // Start of work RAM
constexpr uint16_t ADDR_ECHO_RAM_START = 0xE000; // Start of echo RAM mirror
constexpr uint16_t ADDR_OAM_END        = 0xFEA0; // End of OAM / start of prohibited area
constexpr uint16_t ADDR_IO_START       = 0xFF00; // Start of I/O registers
constexpr uint16_t ADDR_HRAM_START     = 0xFF80; // Start of high RAM

// MBC write-decode range boundaries
constexpr uint16_t MBC_RAM_ENABLE_END  = 0x2000; // End of RAM enable register range
constexpr uint16_t MBC_RAM_BANK_END    = 0x6000; // End of RAM bank number register range

// ROM and RAM bank sizes
constexpr uint32_t ROM_BANK_SIZE       = 0x4000; // 16 KiB per ROM bank
constexpr uint32_t RAM_BANK_SIZE       = 0x2000; // 8 KiB per RAM bank

// ROM header field offsets
constexpr uint16_t ROM_HEADER_CART_TYPE = 0x0147; // Cartridge type
constexpr uint16_t ROM_HEADER_ROM_SIZE  = 0x0148; // ROM size code
constexpr uint16_t ROM_HEADER_RAM_SIZE  = 0x0149; // RAM size code

// MBC2 built-in RAM
constexpr uint16_t MBC2_RAM_SIZE       = 512;    // MBC2 has 512 half-byte entries
constexpr uint16_t MBC2_RAM_MASK       = 0x01FF; // MBC2 RAM address mask

// OAM DMA
constexpr int      OAM_DMA_LENGTH      = 0xA0;   // 160 bytes transferred by OAM DMA

// Memory addresses
constexpr uint16_t OAM_BASE  = 0xFE00; // Object Attribute Memory

// Interrupt bits
constexpr uint8_t INT_VBLANK = 0x01; // Bit 0: VBlank
constexpr uint8_t INT_LCD_STAT = 0x02; // Bit 1: LCD STAT
constexpr uint8_t INT_TIMER = 0x04; // Bit 2: Timer
constexpr uint8_t INT_SERIAL = 0x08; // Bit 3: Serial
constexpr uint8_t INT_JOYPAD = 0x10; // Bit 4: Joypad

// Interrupt vectors
constexpr uint16_t INT_VECTOR_VBLANK = 0x0040;
constexpr uint16_t INT_VECTOR_LCD_STAT = 0x0048;
constexpr uint16_t INT_VECTOR_TIMER = 0x0050;
constexpr uint16_t INT_VECTOR_SERIAL = 0x0058;
constexpr uint16_t INT_VECTOR_JOYPAD = 0x0060;
