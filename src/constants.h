#pragma once
#include <cstdint>

// Game Boy timing constants
constexpr int CPU_FREQUENCY = 4194304;        // 4.194304 MHz (cycles per second)
constexpr double FRAME_RATE = 59.7;           // ~59.7 Hz (frames per second)
constexpr int CYCLES_PER_FRAME = static_cast<int>(CPU_FREQUENCY / FRAME_RATE);  // ~70224 cycles

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

// CGB registers
constexpr uint16_t IO_VBK    = 0xFF4F; // VRAM Bank
constexpr uint16_t IO_SVBK   = 0xFF70; // WRAM Bank
constexpr uint16_t IO_HDMA5  = 0xFF55; // HDMA5
constexpr uint16_t IO_OPRI   = 0xFF6C; // CGB (OAM-order) object priority
constexpr uint16_t IO_KEY1   = 0xFF4D; // CGB Speed Switch
constexpr uint16_t IO_BGPI   = 0xFF68; // BG Palette Index
constexpr uint16_t IO_BGPD   = 0xFF69; // BG Palette Data
constexpr uint16_t IO_OBPI   = 0xFF6A; // OBJ Palette Index
constexpr uint16_t IO_OBPD   = 0xFF6B; // OBJ Palette Data
constexpr uint16_t IO_HDMA1  = 0xFF51; // VRAM DMA source high
constexpr uint16_t IO_HDMA2  = 0xFF52; // VRAM DMA source low
constexpr uint16_t IO_HDMA3  = 0xFF53; // VRAM DMA destination high
constexpr uint16_t IO_HDMA4  = 0xFF54; // VRAM DMA destination low
constexpr uint16_t IO_RP     = 0xFF56; // Infrared port

// CGB register bit layouts
constexpr uint8_t VBK_UNUSED_BITS      = 0xFE; // VBK bits 1-7 always read as 1
constexpr uint8_t VBK_BANK_MASK        = 0x01;
constexpr uint8_t SVBK_UNUSED_BITS     = 0xF8; // SVBK bits 3-7 always read as 1
constexpr uint8_t SVBK_BANK_MASK       = 0x07;
constexpr uint8_t KEY1_DOUBLE_SPEED    = 0x80; // Read-only: currently in double speed
constexpr uint8_t KEY1_ARMED           = 0x01; // Speed switch armed (switches on the next STOP)
constexpr uint8_t KEY1_UNUSED_BITS     = 0x7E; // KEY1 bits 1-6 always read as 1
constexpr uint8_t HDMA5_INACTIVE_BIT   = 0x80; // HDMA5 bit 7 reads 1 when no transfer is active
constexpr uint8_t HDMA5_HBLANK_MODE    = 0x80; // HDMA5 bit 7 on write: 1 = HBlank DMA, 0 = general-purpose DMA
constexpr uint8_t HDMA_LENGTH_MASK     = 0x7F; // HDMA5 bits 0-6: blocks remaining minus 1
constexpr uint8_t HDMA_ADDR_ALIGN_MASK = 0xF0; // HDMA addresses are 16-byte aligned: low 4 bits ignored
constexpr uint8_t HDMA_DEST_HIGH_MASK  = 0x1F; // HDMA destination high byte is only 5 bits ($8000-$9FFF)

// CGB colour RAM (BG and OBJ palette RAM)
constexpr int COLOURS_PER_PALETTE = 4;
constexpr int CRAM_COLOUR_BYTES   = 2;                                         // RGB555, little-endian
constexpr int CRAM_PALETTE_BYTES  = COLOURS_PER_PALETTE * CRAM_COLOUR_BYTES;   // 8
constexpr int CRAM_SIZE           = 8 * CRAM_PALETTE_BYTES;                    // 8 palettes: 64 bytes
constexpr int CRAM_COLOURS        = CRAM_SIZE / CRAM_COLOUR_BYTES;             // 32
constexpr uint8_t CRAM_INDEX_MASK     = 0x3F; // BGPI/OBPI bits 0-5: byte index into CRAM
constexpr uint8_t CRAM_INDEX_UNUSED   = 0x40; // BGPI/OBPI bit 6 always reads as 1
constexpr uint8_t CRAM_AUTO_INCREMENT = 0x80; // BGPI/OBPI bit 7: increment the index after each data write
constexpr uint16_t RGB555_CHANNEL_MASK = 0x1F;                                 // 5 bits per channel
constexpr uint16_t CGB_WHITE_RGB555    = 0x7FFF;                               // All three channels at maximum

// CGB speed switch
constexpr int SPEED_SWITCH_STALL_CYCLES = 8200; // Real-time T-cycles the CPU is stalled while switching speed

// VRAM DMA (HDMA/GDMA) copies in blocks of this many bytes
constexpr int HDMA_BLOCK_SIZE = 0x10;

// Post-boot CPU register values (the boot ROM is skipped)
constexpr uint16_t POST_BOOT_AF_DMG        = 0x01B0;
constexpr uint16_t POST_BOOT_BC_DMG        = 0x0013;
constexpr uint16_t POST_BOOT_DE_DMG        = 0x00D8;
constexpr uint16_t POST_BOOT_HL_DMG        = 0x014D;
constexpr uint16_t POST_BOOT_AF_CGB        = 0x1180; // A = $11 is how a game detects CGB hardware
constexpr uint16_t POST_BOOT_BC_CGB        = 0x0000;
constexpr uint16_t POST_BOOT_DE_CGB        = 0xFF56;
constexpr uint16_t POST_BOOT_HL_CGB        = 0x000D;
constexpr uint16_t POST_BOOT_DE_CGB_COMPAT = 0x0008; // DMG cartridge on CGB hardware
constexpr uint16_t POST_BOOT_HL_CGB_COMPAT = 0x007C;
constexpr uint16_t POST_BOOT_HL_CGB_COMPAT_LOGO = 0x991A; // Titles whose checksum needs the logo tilemap

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
constexpr uint8_t MBC_RAM_ENABLE       = 0x0A;   // Value to enable RAM on MBC1/MBC2/MBC3


// ROM and RAM bank sizes
constexpr uint32_t ROM_BANK_SIZE       = 0x4000; // 16 KiB per ROM bank
constexpr uint32_t RAM_BANK_SIZE       = 0x2000; // 8 KiB per RAM bank
constexpr uint32_t VRAM_BANK_SIZE      = 0x2000; // 8 KiB per VRAM bank (CGB has two)
constexpr uint32_t WRAM_BANK_SIZE      = 0x1000; // 4 KiB per WRAM bank (CGB has eight)
constexpr uint16_t ADDR_WRAM_BANKED_START = 0xD000; // Start of the switchable WRAM bank
constexpr uint16_t ECHO_RAM_OFFSET     = 0x2000; // Echo RAM mirrors WRAM at this offset

// ROM header field offsets
constexpr uint16_t ROM_HEADER_TITLE         = 0x0134; // Game title
constexpr uint16_t ROM_HEADER_TITLE_LENGTH  = 16;     // Title bytes used for the CGB compat palette checksum
constexpr uint16_t ROM_HEADER_FOURTH_LETTER = 0x0137; // 4th title letter, disambiguates some compat palettes
constexpr uint16_t ROM_HEADER_CGB_FLAG      = 0x0143; // CGB flag
constexpr uint16_t ROM_HEADER_NEW_LICENSEE  = 0x0144; // New licensee code (2 ASCII chars)
constexpr uint16_t ROM_HEADER_CART_TYPE     = 0x0147; // Cartridge type
constexpr uint16_t ROM_HEADER_ROM_SIZE      = 0x0148; // ROM size code
constexpr uint16_t ROM_HEADER_RAM_SIZE      = 0x0149; // RAM size code
constexpr uint16_t ROM_HEADER_OLD_LICENSEE  = 0x014B; // Old licensee code
constexpr uint16_t ROM_HEADER_END           = 0x0150; // First byte after the header

constexpr uint8_t CGB_FLAG_ENHANCED = 0x80; // Runs on DMG and CGB
constexpr uint8_t CGB_FLAG_ONLY     = 0xC0; // Runs on CGB only
constexpr uint8_t LICENSEE_USE_NEW_FIELD = 0x33; // Old licensee code meaning "see the new licensee field"
constexpr uint8_t LICENSEE_NINTENDO_OLD  = 0x01; // Nintendo as an old licensee code
// Nintendo as a new licensee code is the ASCII string "01"

// MBC2 built-in RAM
constexpr uint16_t MBC2_RAM_SIZE       = 512;    // MBC2 has 512 half-byte entries
constexpr uint16_t MBC2_RAM_MASK       = 0x01FF; // MBC2 RAM address mask

// MBC5
constexpr uint16_t MBC5_ROM_HIGH_ADDR = 0x3000; // MBC5 ROM bank register low/high split

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
constexpr uint8_t INT_ALL_MASK = 0x1F; // Mask for all interrupt bits

// Interrupt vectors
constexpr uint16_t INT_VECTOR_VBLANK = 0x0040;
constexpr uint16_t INT_VECTOR_LCD_STAT = 0x0048;
constexpr uint16_t INT_VECTOR_TIMER = 0x0050;
constexpr uint16_t INT_VECTOR_SERIAL = 0x0058;
constexpr uint16_t INT_VECTOR_JOYPAD = 0x0060;

// CPU constants
constexpr uint16_t CPU_INITIAL_PC = 0x0100; // Initial program counter value after boot ROM
constexpr uint16_t CPU_INITIAL_SP = 0xFFFE; // Initial stack pointer value after boot ROM

// Joypad state bits (0 = pressed). Bits 0-3: A, B, Select, Start. Bits 4-7: Right, Left, Up, Down.
constexpr uint8_t JOYPAD_A      = 0x01;
constexpr uint8_t JOYPAD_B      = 0x02;
constexpr uint8_t JOYPAD_SELECT = 0x04;
constexpr uint8_t JOYPAD_START  = 0x08;
constexpr uint8_t JOYPAD_RIGHT  = 0x10;
constexpr uint8_t JOYPAD_LEFT   = 0x20;
constexpr uint8_t JOYPAD_UP     = 0x40;
constexpr uint8_t JOYPAD_DOWN   = 0x80;

