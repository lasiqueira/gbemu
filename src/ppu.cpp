#include "ppu.h"
#include "memory.h"
#include "gameboy.h"
#include <cstring>
#include <print>

// PPU I/O Register addresses
constexpr uint16_t LCDC = 0xFF40; // LCD Control
constexpr uint16_t STAT = 0xFF41; // LCD Status
constexpr uint16_t SCY = 0xFF42;  // Scroll Y
constexpr uint16_t SCX = 0xFF43;  // Scroll X
constexpr uint16_t LY = 0xFF44;   // LCD Y-Coordinate
constexpr uint16_t LYC = 0xFF45;  // LY Compare
constexpr uint16_t BGP = 0xFF47;  // BG Palette Data
constexpr uint16_t OBP0 = 0xFF48; // Object Palette 0 Data
constexpr uint16_t OBP1 = 0xFF49; // Object Palette 1 Data
constexpr uint16_t WY = 0xFF4A;   // Window Y Position
constexpr uint16_t WX = 0xFF4B;   // Window X Position

PPU::PPU() : mode(PPUMode::OAMSearch), mode_cycles(0), scanline(0), window_line_counter(0), frame_ready(false)
{
    framebuffer.fill(0);
    rgba_buffer.fill(0);
}

void PPU::step(int cycles, Memory& memory)
{
    uint8_t lcdc = memory.read(LCDC);
    
    // If LCD is disabled, reset PPU state and output white screen
    if (!(lcdc & LCDC_ENABLE))
    {
        mode = PPUMode::OAMSearch;
        mode_cycles = 0;
        scanline = 0;
        window_line_counter = 0;
        memory.write(LY, 0);
        
        // Fill framebuffer with white (color 0 = lightest)
        framebuffer.fill(0);
        update_rgba_buffer();
        frame_ready = true;
        return;
    }
    
    mode_cycles += cycles;
    
    switch (mode)
    {
        case PPUMode::OAMSearch:
            if (mode_cycles >= MODE_2_CYCLES)
            {
                mode_cycles -= MODE_2_CYCLES;
                set_mode(PPUMode::Drawing, memory);
            }
            break;
            
        case PPUMode::Drawing:
            if (mode_cycles >= MODE_3_CYCLES)
            {
                mode_cycles -= MODE_3_CYCLES;
                render_scanline(memory);
                set_mode(PPUMode::HBlank, memory);
            }
            break;
            
        case PPUMode::HBlank:
            if (mode_cycles >= MODE_0_CYCLES)
            {
                mode_cycles -= MODE_0_CYCLES;
                scanline++;
                memory.write(LY, scanline);
                
                // Check LYC=LY coincidence
                uint8_t lyc = memory.read(LYC);
                uint8_t stat = memory.read(STAT);
                if (scanline == lyc)
                {
                    stat |= STAT_LYC_FLAG;
                    if (stat & STAT_LYC_INT)
                    {
                        request_interrupt(memory, INT_LCD_STAT);
                    }
                }
                else
                {
                    stat &= ~STAT_LYC_FLAG;
                }
                memory.write(STAT, stat);
                
                if (scanline >= SCREEN_HEIGHT)
                {
                    // Enter V-Blank
                    set_mode(PPUMode::VBlank, memory);
                    request_interrupt(memory, INT_VBLANK);
                    update_rgba_buffer();
                    frame_ready = true;
                }
                else
                {
                    set_mode(PPUMode::OAMSearch, memory);
                }
            }
            break;
            
        case PPUMode::VBlank:
            if (mode_cycles >= SCANLINE_CYCLES)
            {
                mode_cycles -= SCANLINE_CYCLES;
                scanline++;
                memory.write(LY, scanline);
                
                // Check LYC=LY coincidence
                uint8_t lyc = memory.read(LYC);
                uint8_t stat = memory.read(STAT);
                if (scanline == lyc)
                {
                    stat |= STAT_LYC_FLAG;
                    if (stat & STAT_LYC_INT)
                    {
                        request_interrupt(memory, INT_LCD_STAT);
                    }
                }
                else
                {
                    stat &= ~STAT_LYC_FLAG;
                }
                memory.write(STAT, stat);
                
                if (scanline > 153)
                {
                    // Start new frame
                    scanline = 0;
                    window_line_counter = 0;
                    memory.write(LY, scanline);
                    set_mode(PPUMode::OAMSearch, memory);
                }
            }
            break;
    }
}

void PPU::render_scanline(Memory& memory)
{
    uint8_t lcdc = memory.read(LCDC);
    
    bool background_enabled = lcdc & LCDC_BG_ENABLE;
    bool window_enabled = lcdc & LCDC_WINDOW_ENABLE;
    // If BG is disabled, render white line
    if (!background_enabled)
    {
        for (int x = 0; x < SCREEN_WIDTH; x++)
        {
            framebuffer[scanline * SCREEN_WIDTH + x] = 0; // Lightest color
        }
        return;
    }
    
    uint8_t scy = memory.read(SCY);
    uint8_t scx = memory.read(SCX);
    // Background
    uint8_t bgp = memory.read(BGP);
    // Window 
    uint8_t wy = memory.read(WY);
    uint8_t wx = memory.read(WX);

    bool window_visible_this_line = window_enabled && (scanline >= wy);

    // Determine tile map and tile data addresses
    uint16_t tile_data_base = (lcdc & LCDC_TILE_DATA) ? 0x8000 : 0x8800;
    bool signed_tile_ids = !(lcdc & LCDC_TILE_DATA);

    // determine tile maps
    uint16_t bg_tile_map = (lcdc & LCDC_BG_MAP) ? 0x9C00 : 0x9800;
    uint16_t window_tile_map = (lcdc & LCDC_WINDOW_MAP) ? 0x9C00 : 0x9800;

    bool window_rendered_this_line = false;
    
    // Render
    for (int x = 0; x < SCREEN_WIDTH; x++)
    {
        uint8_t palette_color;
        
        bool draw_window = window_visible_this_line && (x >= (wx - 7));
        if(draw_window)
        {
            uint8_t pixel_x = x - (wx - 7);
            uint8_t pixel_y = window_line_counter;

            palette_color = get_tile_pixel(
                pixel_x,
                pixel_y,
                window_tile_map, 
                tile_data_base, 
                signed_tile_ids, 
                bgp,
                memory
            );
            
            window_rendered_this_line = true;
        } 
        else 
        {
            // Calculate tile coordinates with scrolling
            uint8_t pixel_y = (scanline + scy) & 0xFF;
            uint8_t pixel_x = (x + scx) & 0xFF;
            
            palette_color = get_tile_pixel(
                pixel_x,
                pixel_y,
                bg_tile_map, 
                tile_data_base, 
                signed_tile_ids, 
                bgp,
                memory
            );
           
        }
         framebuffer[scanline * SCREEN_WIDTH + x] = palette_color;
    }

    if(window_rendered_this_line)
    {
        window_line_counter++;
    }
}

void PPU::update_rgba_buffer()
{
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
    {
        uint8_t palette_idx = framebuffer[i];
        const Color& color = GB_PALETTE[palette_idx];
        
        rgba_buffer[i * 4 + 0] = color.r;
        rgba_buffer[i * 4 + 1] = color.g;
        rgba_buffer[i * 4 + 2] = color.b;
        rgba_buffer[i * 4 + 3] = 255; // Alpha
    }
}

void PPU::set_mode(PPUMode new_mode, Memory& memory)
{
    mode = new_mode;
    
    uint8_t stat = memory.read(STAT);
    stat = (stat & ~STAT_MODE_MASK) | static_cast<uint8_t>(new_mode);
    memory.write(STAT, stat);
    
    // Request STAT interrupt if enabled
    bool request_stat_int = false;
    switch (new_mode)
    {
        case PPUMode::HBlank:
            request_stat_int = stat & STAT_MODE0_INT;
            break;
        case PPUMode::VBlank:
            request_stat_int = stat & STAT_MODE1_INT;
            break;
        case PPUMode::OAMSearch:
            request_stat_int = stat & STAT_MODE2_INT;
            break;
        case PPUMode::Drawing:
            break;
    }
    
    if (request_stat_int)
    {
        request_interrupt(memory, INT_LCD_STAT);
    }
}

void PPU::request_interrupt(Memory& memory, uint8_t interrupt_bit)
{
    uint8_t if_reg = memory.read(IO_IF);
    if_reg |= interrupt_bit;
    memory.write(IO_IF, if_reg);
}

uint8_t PPU::get_tile_pixel(uint8_t pixel_x, uint8_t pixel_y, uint16_t tile_map_base, uint16_t tile_data_base, bool signed_tile_ids, uint8_t palette, Memory& memory)
{
   
    uint8_t tile_y = pixel_y / 8;
    uint8_t tile_x = pixel_x / 8;
    uint16_t tile_map_addr = tile_map_base + tile_y * 32 + tile_x;
    
    uint8_t tile_id = memory.read(tile_map_addr);
    
    // Get tile data address
    uint16_t tile_addr;
    if (signed_tile_ids)
    {
        int8_t signed_id = static_cast<int8_t>(tile_id);
        tile_addr = tile_data_base + (signed_id + 128) * 16;
    }
    else
    {
        tile_addr = tile_data_base + tile_id * 16;
    }
    
    // Get pixel within tile
    uint8_t tile_pixel_y = pixel_y % 8;
    uint8_t tile_pixel_x = pixel_x % 8;
    
    // Each tile row is 2 bytes
    uint16_t tile_row_addr = tile_addr + tile_pixel_y * 2;
    uint8_t byte1 = memory.read(tile_row_addr);
    uint8_t byte2 = memory.read(tile_row_addr + 1);
    
    // Get color from pixel (bit 7 = leftmost pixel)
    int bit_pos = 7 - tile_pixel_x;
    uint8_t color_id = ((byte2 >> bit_pos) & 1) << 1 | ((byte1 >> bit_pos) & 1);
    
    // Apply palette
    return (palette >> (color_id * 2)) & 0x03;
}
