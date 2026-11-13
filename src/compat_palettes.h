#pragma once
#include <cstdint>
#include <vector>

struct CompatPalettes
{
    uint16_t bg[4]; // RGB555
    uint16_t obj0[4];
    uint16_t obj1[4];
    uint8_t checksum; // title checksum, or 0 if not Nintend-licensed; goes into B
};

CompatPalettes pick_compat_palettes(const std::vector<uint8_t>& rom);