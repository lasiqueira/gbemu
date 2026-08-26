#include "ppu.h"
#include "memory.h"
#include "constants.h"
#include <algorithm>

void PPU::step(int cycles, Memory& memory)
{
    uint8_t lcdc = memory.read_io_raw(IO_LCDC);
    
    // If LCD is disabled, reset PPU state and output white screen
    if (!(lcdc & LCDC_ENABLE))
    {
        if(!lcd_off)
        {
            lcd_off = true;
            mode = PPUMode::OAMSearch;
            mode_cycles = 0;
            scanline = 0;
            window_line_counter = 0;
            memory.write_io_raw(IO_LY, 0);
            framebuffer.fill(0);
            update_rgba_buffer();
            frame_ready = true;
        }
        return;
    }
    lcd_off = false;
    mode_cycles += cycles;
    
    switch (mode)
    {
        case PPUMode::OAMSearch:
            if (mode_cycles >= MODE_2_CYCLES)
            {
                mode_cycles -= MODE_2_CYCLES;
                scan_oam(memory);
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
                memory.write_io_raw(IO_LY, scanline);
                
                // Check IO_LYC=IO_LY coincidence
                uint8_t lyc = memory.read_io_raw(IO_LYC);
                uint8_t stat = memory.read_io_raw(IO_STAT);
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
                memory.write_io_raw(IO_STAT, stat);
                
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
                memory.write_io_raw(IO_LY, scanline);
                
                // Check IO_LYC=IO_LY coincidence
                uint8_t lyc = memory.read_io_raw(IO_LYC);
                uint8_t stat = memory.read_io_raw(IO_STAT);
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
                memory.write_io_raw(IO_STAT, stat);
                
                if (scanline >= SCANLINES_PER_FRAME)
                {
                    // Start new frame
                    scanline = 0;
                    window_line_counter = 0;
                    memory.write_io_raw(IO_LY, scanline);
                    set_mode(PPUMode::OAMSearch, memory);
                }
            }
            break;
    }
}

void PPU::render_scanline(Memory& memory)
{
    uint8_t lcdc = memory.read_io_raw(IO_LCDC);
    
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
    
    uint8_t scy = memory.read_io_raw(IO_SCY);
    uint8_t scx = memory.read_io_raw(IO_SCX);
    // Background
    uint8_t bgp = memory.read_io_raw(IO_BGP);
    // Window 
    uint8_t wy = memory.read_io_raw(IO_WY);
    uint8_t wx = memory.read_io_raw(IO_WX);

    bool window_visible_this_line = window_enabled && (scanline >= wy);

    // Determine tile map and tile data addresses
    uint16_t tile_data_base = (lcdc & LCDC_TILE_DATA) ? TILE_DATA_BASE_UNSIGNED : TILE_DATA_BASE_SIGNED;
    bool signed_tile_ids = !(lcdc & LCDC_TILE_DATA);

    // determine tile maps
    uint16_t bg_tile_map = (lcdc & LCDC_BG_MAP) ? TILE_MAP_BASE_1 : TILE_MAP_BASE_0;
    uint16_t window_tile_map = (lcdc & LCDC_WINDOW_MAP) ? TILE_MAP_BASE_1 : TILE_MAP_BASE_0;

    bool window_rendered_this_line = false;
    
    TileBytes cached_tile_bytes = {0, 0};
    int cached_tile_x = -1;
    bool cached_was_window = false;

    int sprite_height = (lcdc & LCDC_OBJ_SIZE) ? 16 : 8;
    for (int i = 0; i < visible_sprite_count; i++)
    {
        const Sprite& sprite = visible_sprites[i];
        int left = sprite.x - SPRITE_X_OFFSET;
        uint8_t palette = (sprite.attributes & SPRITE_PALETTE) ? memory.read_io_raw(IO_OBP1) : memory.read_io_raw(IO_OBP0);
        for (int px = std::max(0, left); px < std::min(SCREEN_WIDTH, left + 8); px++)
        {
            if(sprite_line[px].present) continue;

            int color = get_sprite_pixel(sprite, px, sprite_height, palette, memory);
            if(color != -1)
            {
                sprite_line[px].color = color;
                sprite_line[px].priority = sprite.attributes;
                sprite_line[px].present = true;
            }
        }
    }

    // Render
    for (int x = 0; x < SCREEN_WIDTH; x++)
    {
        
        bool draw_window = window_visible_this_line && (x >= (wx - WINDOW_X_OFFSET));
        uint8_t pixel_x = draw_window ? x - (wx - WINDOW_X_OFFSET) : (x + scx) & 0xFF;
        uint8_t pixel_y = draw_window ? window_line_counter : (scanline + scy) & 0xFF;
        int tile_x = pixel_x / 8;
        
        if(draw_window)
        {
            window_rendered_this_line = true;
        } 
        
        if (tile_x != cached_tile_x || draw_window != cached_was_window)
        {
            cached_tile_bytes = fetch_tile_row(pixel_x, pixel_y, draw_window ? window_tile_map : bg_tile_map, tile_data_base, signed_tile_ids, memory);
            cached_tile_x = tile_x;
            cached_was_window = draw_window;
        }

        int bit_pos = 7 - (pixel_x % 8);
        uint8_t color_id = ((cached_tile_bytes.byte2 >> bit_pos) & 1) << 1 | ((cached_tile_bytes.byte1 >> bit_pos) & 1);

        uint8_t bg_color = (bgp >> (color_id * 2)) & 0x03;

        int sprite_color = -1;
        uint8_t sprite_priority = 0;
        
        if (sprite_line[x].present)
        {
            sprite_color = sprite_line[x].color;
            sprite_priority = sprite_line[x].priority;
            sprite_line[x].present = false; // Reset for next scanline
        }

        uint8_t final_color;
        if(sprite_color == -1)
        {
            final_color = bg_color;
        }
        else if(sprite_priority & SPRITE_PRIORITY)
        {
            // Sprite behind BG, but BG color 0 is transparent
            if(bg_color == 0)
            {
                // BG color 0 is transparent, so sprite is visible
                final_color = sprite_color;
            }
            else
            {
                // BG color 1-3 are opaque, so sprite is behind BG
                final_color = bg_color;
            }
        }
        else 
        {
            // Sprite above BG
            final_color = sprite_color; 
        }      

        framebuffer[scanline * SCREEN_WIDTH + x] = final_color;
    }

    if(window_rendered_this_line)
    {
        window_line_counter++;
    }
}

void PPU::update_rgba_buffer()
{
    uint32_t* dst = rgba_buffer.data();
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
    {
        dst[i] = GB_PALETTE_RGBA32[framebuffer[i]];
    }
}

void PPU::set_mode(PPUMode new_mode, Memory& memory)
{
    mode = new_mode;
    
    uint8_t stat = memory.read_io_raw(IO_STAT);
    stat = (stat & ~STAT_MODE_MASK) | static_cast<uint8_t>(new_mode);
    memory.write_io_raw(IO_STAT, stat);
    
    // Request IO_STAT interrupt if enabled
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
    uint8_t if_reg = memory.read_io_raw(IO_IF);
    if_reg |= interrupt_bit;
    memory.write_io_raw(IO_IF, if_reg);
}

void PPU::scan_oam(Memory& memory)
{
    visible_sprite_count = 0;

    uint8_t lcdc = memory.read_io_raw(IO_LCDC);

    // If sprites are disabled, skip scanning OAM
    if(!(lcdc & LCDC_OBJ_ENABLE))
    {
        return; // Sprites are disabled
    }

    int sprite_height = (lcdc & LCDC_OBJ_SIZE) ? 16 : 8;

    for(int i = 0; i < OAM_SPRITE_COUNT; i++)
    {
        // Each sprite takes 4 bytes in OAM
        uint16_t sprite_addr = OAM_BASE + (i * 4);

        // Read sprite attributes from OAM
        uint8_t y = memory.read_oam(sprite_addr);
        uint8_t x = memory.read_oam(sprite_addr + 1);
        uint8_t tile = memory.read_oam(sprite_addr + 2);
        uint8_t attributes = memory.read_oam(sprite_addr + 3);

        if(y == 0 || y >= SCREEN_HEIGHT + SPRITE_Y_OFFSET) continue; // Sprite is off-screen vertically

        int sprite_top = y - SPRITE_Y_OFFSET; // Adjust for sprite offset
        int sprite_bottom = sprite_top + sprite_height;

        if(scanline >= sprite_top && scanline < sprite_bottom)
        {
            visible_sprites[visible_sprite_count++] = {y, x, tile, attributes, static_cast<uint8_t>(i)};

            if(visible_sprite_count >= MAX_SPRITES_PER_LINE)
            {
                break; // Reached max sprites for this line
            }
        }
    }

    // DMG priority: lower X wins; ties broken by lower OAM index (already in order)
    std::sort(visible_sprites.begin(), visible_sprites.begin() + visible_sprite_count,
        [](const Sprite& a, const Sprite& b) {
            if(a.x != b.x) return a.x < b.x;
            return a.oam_index < b.oam_index;
        });
}

int PPU::get_sprite_pixel(const Sprite& sprite, int screen_x, int sprite_height, uint8_t palette, Memory& memory)
{
    int sprite_left = sprite.x - SPRITE_X_OFFSET;
    int pixel_x = screen_x - sprite_left;
    int pixel_y = scanline - (sprite.y - SPRITE_Y_OFFSET); // Adjust for sprite offset

    // Handle Y flip
    if(sprite.attributes & SPRITE_FLIP_Y)
    {
        pixel_y = (sprite_height - 1) - pixel_y;
    }

    // Handle X flip
    if(sprite.attributes & SPRITE_FLIP_X)
    {
        pixel_x = 7 - pixel_x;
    }

    // Determine which tile to use
    uint8_t tile_index = sprite.tile_index;
    if(sprite_height == 16)
    {
        if(pixel_y >= 8)
        {
            // Bottom half
            tile_index = (sprite.tile_index & 0xFE) + 1;
            pixel_y -= 8;
        }
        else
        {
            // Top half
            tile_index = sprite.tile_index & 0xFE;
        }
    }

    // Get tile data address
    uint16_t tile_addr = ADDR_VRAM_START + (tile_index * BYTES_PER_TILE);
    // Each tile row is 2 bytes
    uint16_t tile_row_addr = tile_addr + (pixel_y * 2);
    uint8_t byte1 = memory.read_vram(tile_row_addr);
    uint8_t byte2 = memory.read_vram(tile_row_addr + 1);

    // Get color from pixel (bit 7 = leftmost pixel)
    int bit_pos = 7 - pixel_x;
    uint8_t color_id = ((byte2 >> bit_pos) & 1) << 1 | ((byte1 >> bit_pos) & 1);

    // Color ID 0 is transparent for sprites
    if(color_id == 0)
    {
        return -1;
    }

    uint8_t palette_color = (palette >> (color_id * 2)) & 0x03;

    return palette_color;
}

TileBytes PPU::fetch_tile_row(uint8_t pixel_x, uint8_t pixel_y, uint16_t tile_map_base, uint16_t tile_data_base, bool signed_tile_ids, Memory& memory)
{
    uint8_t tile_y = pixel_y / 8;
    uint8_t tile_x = pixel_x / 8;
    uint16_t tile_map_addr = tile_map_base + tile_y * TILE_MAP_COLS + tile_x;
    
    uint8_t tile_id = memory.read_vram(tile_map_addr);
    
    // Get tile data address
    uint16_t tile_addr;
    if (signed_tile_ids)
    {
        int8_t signed_id = static_cast<int8_t>(tile_id);
        tile_addr = tile_data_base + (signed_id + 128) * BYTES_PER_TILE;
    }
    else
    {
        tile_addr = tile_data_base + tile_id * BYTES_PER_TILE;
    }
    
    // Get pixel within tile
    uint8_t tile_pixel_y = pixel_y % 8;
    
    // Each tile row is 2 bytes
    uint16_t tile_row_addr = tile_addr + tile_pixel_y * 2;
    TileBytes row_bytes;
    row_bytes.byte1 = memory.read_vram(tile_row_addr);
    row_bytes.byte2 = memory.read_vram(tile_row_addr + 1);
    
    return row_bytes;
}
