#pragma once
#include <cstdint>
#include <vector>

struct CompatPalettes
{
    uint16_t bg[4]; // RGB555
    uint16_t obj0[4];
    uint16_t obj1[4];
    uint8_t checksum; // title checksum, or 0 if not Nintendo-licensed; goes into B
    bool logo_tilemap; // the boot ROM also writes the Nintendo logo tilemap for this title; changes HL
};

CompatPalettes pick_compat_palettes(const std::vector<uint8_t>& rom);