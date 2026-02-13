#pragma once
#include <cstdint>
#include <array>

// Forward declaration
struct Memory;

// Game Boy display dimensions
constexpr int SCREEN_WIDTH = 160;
constexpr int SCREEN_HEIGHT = 144;

// PPU timing constants (in CPU cycles)
constexpr int MODE_2_CYCLES = 80;   // OAM Search
constexpr int MODE_3_CYCLES = 172;  // Drawing pixels
constexpr int MODE_0_CYCLES = 204;  // H-Blank
constexpr int SCANLINE_CYCLES = 456; // Total cycles per scanline
constexpr int VBLANK_SCANLINES = 10; // Scanlines in V-Blank

// LCD Control (LCDC) register bits
constexpr uint8_t LCDC_ENABLE = 0x80;        // Bit 7: LCD/PPU Enable
constexpr uint8_t LCDC_WINDOW_MAP = 0x40;    // Bit 6: Window tile map area
constexpr uint8_t LCDC_WINDOW_ENABLE = 0x20; // Bit 5: Window enable
constexpr uint8_t LCDC_TILE_DATA = 0x10;     // Bit 4: BG & Window tile data area
constexpr uint8_t LCDC_BG_MAP = 0x08;        // Bit 3: BG tile map area
constexpr uint8_t LCDC_OBJ_SIZE = 0x04;      // Bit 2: OBJ size (0=8x8, 1=8x16)
constexpr uint8_t LCDC_OBJ_ENABLE = 0x02;    // Bit 1: OBJ enable
constexpr uint8_t LCDC_BG_ENABLE = 0x01;     // Bit 0: BG & Window enable/priority

// LCD Status (STAT) register bits
constexpr uint8_t STAT_LYC_INT = 0x40;       // Bit 6: LYC=LY interrupt
constexpr uint8_t STAT_MODE2_INT = 0x20;     // Bit 5: Mode 2 OAM interrupt
constexpr uint8_t STAT_MODE1_INT = 0x10;     // Bit 4: Mode 1 V-Blank interrupt
constexpr uint8_t STAT_MODE0_INT = 0x08;     // Bit 3: Mode 0 H-Blank interrupt
constexpr uint8_t STAT_LYC_FLAG = 0x04;      // Bit 2: LYC=LY flag
constexpr uint8_t STAT_MODE_MASK = 0x03;     // Bits 0-1: Mode flag

// Sprite/OAM constants
constexpr int MAX_SPRITES_PER_LINE = 10; // Maximum sprites that can be rendered on a single scanline
constexpr int OAM_SPRITE_COUNT = 40;     // Total sprites in OAM (Object Attribute Memory)

// Sprite attribute flags
constexpr uint8_t SPRITE_PRIORITY = 0x80; // Bit 7: Sprite-to-background priority (0=OBJ above BG, 1=OBJ behind BG color 1-3)
constexpr uint8_t SPRITE_FLIP_Y = 0x40;   // Bit 6: Y flip
constexpr uint8_t SPRITE_FLIP_X = 0x20;   // Bit 5: X flip
constexpr uint8_t SPRITE_PALETTE = 0x10;   // Bit 4: Palette number (0=OBP0, 1=OBP1)

// Sprite/Object data for rendering
struct Sprite
{
    uint8_t y;       // Y position on screen (minus 16)
    uint8_t x;       // X position on screen (minus 8)
    uint8_t tile_index; // Tile index in memory
    uint8_t attributes; // Attribute flags (priority, flip, palette)
    uint8_t oam_index; // Original index in OAM (for priority)
};

// PPU modes
enum class PPUMode : uint8_t
{
    HBlank = 0,     // Mode 0: H-Blank
    VBlank = 1,     // Mode 1: V-Blank
    OAMSearch = 2,  // Mode 2: Searching OAM
    Drawing = 3     // Mode 3: Transferring data to LCD
};

// Color palette (classic Game Boy colors)
struct Color
{
    uint8_t r, g, b;
};

constexpr Color GB_PALETTE[4] = {
    {0x9B, 0xBC, 0x0F},  // Lightest
    {0x8B, 0xAC, 0x0F},  // Light
    {0x30, 0x62, 0x30},  // Dark
    {0x0F, 0x38, 0x0F}   // Darkest
};

struct PPU
{
    // Framebuffer: 160x144 pixels, each pixel is a palette index (0-3)
    std::array<uint8_t, SCREEN_WIDTH * SCREEN_HEIGHT> framebuffer;
    
    // RGBA framebuffer for rendering (4 bytes per pixel)
    std::array<uint8_t, SCREEN_WIDTH * SCREEN_HEIGHT * 4> rgba_buffer;

    std::array<Sprite, MAX_SPRITES_PER_LINE> visible_sprites; // Sprites visible on the current scanline
   
    
    PPUMode mode;
    int mode_cycles;      // Cycles spent in current mode
    int scanline;         // Current scanline (0-153)
    int window_line_counter; //Track which window line to draw
    int visible_sprite_count; // Number of sprites visible on the current scanline
    
    bool frame_ready;     // True when a frame is complete and ready to display
    
    PPU();
    
    // Update PPU state for the given number of cycles
    void step(int cycles, Memory& memory);
    
    // Render the current scanline
    void render_scanline(Memory& memory);
    
    // Convert framebuffer to RGBA format
    void update_rgba_buffer();
    
    // Set PPU mode and update STAT register
    void set_mode(PPUMode new_mode, Memory& memory);
    
    // Request interrupt
    void request_interrupt(Memory& memory, uint8_t interrupt_bit);

    // Get the color index of a pixel from a tile, given its coordinates and tile data
    uint8_t get_tile_pixel(uint8_t pixel_x, uint8_t pixel_y, uint16_t tile_map_base, uint16_t tile_data_base, bool signed_tile_ids, uint8_t palette, Memory& memory);

    // Scan OAM for sprites visible on the current scanline and populate visible_sprites array
    void scan_oam(Memory& memory);

    int get_sprite_pixel(const Sprite& sprite, int screen_x, Memory& memory);
};
