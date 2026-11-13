#pragma once
#include <cstdint>

// CGB boot ROM data used to colourise DMG cartridges (title checksum -> palettes).
//
// Source: the original CGB boot ROM as disassembled in ISSOtm's gb-bootroms (src/cgb.asm,
// the disassembly Pan Docs links from "Power-Up Sequence"). Every table was cross-checked
// against SameBoy's BootROMs/cgb_boot.asm: for all 94 palette IDs both sources resolve to
// identical (OBJ0, OBJ1, BG) colours.
//
// SameBoy is licensed under the Expat (MIT) licence:
//   Copyright (c) 2015-2026 Lior Halphon
// Permission notice: https://github.com/LIJI32/SameBoy/blob/master/LICENSE
//
// This file contains data only. The lookup algorithm lives in compat_palettes.cpp.
namespace compat_palette_data
{
// Title checksums. Entries [0, FIRST_AMBIGUOUS_CHECKSUM) identify one game each (the index is the
// palette ID). Entries from FIRST_AMBIGUOUS_CHECKSUM on are shared by several games and are
// disambiguated by the 4th title letter (see FOURTH_LETTERS). Index 0 is the "default" entry.
constexpr int FIRST_AMBIGUOUS_CHECKSUM = 65;
constexpr int TITLE_CHECKSUM_COUNT = 79;
constexpr uint8_t TITLE_CHECKSUMS[TITLE_CHECKSUM_COUNT] = {
    0x00, 0x88, 0x16, 0x36, 0xD1, 0xDB, 0xF2, 0x3C, 0x8C, 0x92, 0x3D, 0x5C, 0x58, 0xC9, 0x3E, 0x70,
    0x1D, 0x59, 0x69, 0x19, 0x35, 0xA8, 0x14, 0xAA, 0x75, 0x95, 0x99, 0x34, 0x6F, 0x15, 0xFF, 0x97,
    0x4B, 0x90, 0x17, 0x10, 0x39, 0xF7, 0xF6, 0xA2, 0x49, 0x4E, 0x43, 0x68, 0xE0, 0x8B, 0xF0, 0xCE,
    0x0C, 0x29, 0xE8, 0xB7, 0x86, 0x9A, 0x52, 0x01, 0x9D, 0x71, 0x9C, 0xBD, 0x5D, 0x6D, 0x67, 0x3F,
    0x6B, 0xB3, 0x46, 0x28, 0xA5, 0xC6, 0xD3, 0x27, 0x61, 0x18, 0x66, 0x6A, 0xBF, 0x0D, 0xF4,
};

// 4th title letter, laid out as rows of FOURTH_LETTER_ROW_LENGTH. A letter in column c, row r
// gives palette ID FIRST_AMBIGUOUS_CHECKSUM + c + FOURTH_LETTER_ROW_LENGTH * r, where c is the
// index of the matching checksum minus FIRST_AMBIGUOUS_CHECKSUM. Letter not found -> palette ID 0.
constexpr int FOURTH_LETTER_ROW_LENGTH = 14;
constexpr int FOURTH_LETTER_COUNT = 29;
constexpr char FOURTH_LETTERS[FOURTH_LETTER_COUNT] = {
    'B', 'E', 'F', 'A', 'A', 'R', 'B', 'E', 'K', 'E', 'K', ' ', 'R', '-', 'U', 'R', 'A', 'R', ' ', 'I', 'N', 'A', 'I', 'L', 'I', 'C', 'E', ' ', 'R',
};

// Titles with these checksums also need the Nintendo logo tilemap in VRAM.
constexpr uint8_t LOGO_TILEMAP_CHECKSUMS[2] = { 0x58, 0x43 };

// Palette ID -> where each of the three palettes starts. Each value is an offset, in colours,
// into PALETTE_COLOURS; the palette is the 4 colours starting there. The offsets are NOT always
// multiples of 4: the boot ROM's palettes overlap, so some games start in the middle of one.
struct PaletteSet
{
    uint8_t obj0;
    uint8_t obj1;
    uint8_t bg;
};
constexpr int PALETTE_ID_COUNT = 94;
constexpr PaletteSet PALETTE_SETS[PALETTE_ID_COUNT] = {
    {  16,  16, 116 }, // 0
    {  36,  36,  36 }, // 1
    {   0,   0,   0 }, // 2
    { 111,  16,  60 }, // 3
    { 111,   0,  56 }, // 4
    {  96,  96,  96 }, // 5
    {  96,  88,  96 }, // 6
    { 112,  16, 112 }, // 7
    {  64,  32,  32 }, // 8
    {   0,   0,   0 }, // 9
    {  16,  16,  72 }, // 10
    {  76,  91,  36 }, // 11
    {  20,  20,  20 }, // 12
    {  64, 112,  40 }, // 13
    {  80,  88,  80 }, // 14
    {  84, 112,  16 }, // 15
    {  76,  76,  36 }, // 16
    {  64,  88,  32 }, // 17
    {  96,  88,  96 }, // 18
    {  16,  16,  80 }, // 19
    {   0,   0,   0 }, // 20
    {  68,  16,  52 }, // 21
    {  12,  16,  16 }, // 22
    {  16, 116, 116 }, // 23
    {   0,   0,   0 }, // 24
    {  72,  88,  72 }, // 25
    {   0,   0,   0 }, // 26
    {  16,  16,  28 }, // 27
    { 104, 104, 104 }, // 28
    {  96,  96,  96 }, // 29
    {  80,  80,  80 }, // 30
    { 112, 112,   0 }, // 31
    {  16,  16,  12 }, // 32
    {  16,  16,  12 }, // 33
    {  16, 112,  12 }, // 34
    { 112,  12,   0 }, // 35
    { 112, 112,   0 }, // 36
    {  12, 112,   0 }, // 37
    { 112,  12,   0 }, // 38
    {  12, 112,   0 }, // 39
    {  76,  91,  36 }, // 40
    {  16,  92, 112 }, // 41
    { 112, 112,   0 }, // 42
    { 112,  12,   0 }, // 43
    {  80,  88,  80 }, // 44
    {  16, 112,  12 }, // 45
    { 111,   0,  56 }, // 46
    { 111,   0,  56 }, // 47
    {   0,   0,   0 }, // 48
    { 112,  12,   0 }, // 49
    { 108, 108, 108 }, // 50
    {   0,   0,   0 }, // 51
    {  68,  16,  52 }, // 52
    {  16,  16,  12 }, // 53
    { 112,  12,   0 }, // 54
    { 112,  12,   0 }, // 55
    {  16,   0,   8 }, // 56
    {  80,  80,  80 }, // 57
    {   8,  68,   8 }, // 58
    {  16,  16,  12 }, // 59
    { 112,  12,   0 }, // 60
    { 112,  12,   0 }, // 61
    {   0,   0,   0 }, // 62
    {  16,  16, 116 }, // 63
    {  68,  88,   8 }, // 64
    {  76,  91,  36 }, // 65
    {  15,  15,  44 }, // 66
    {  16,  16,  12 }, // 67
    { 108, 108, 108 }, // 68
    {  64,  88,  32 }, // 69
    {  16,   8,   8 }, // 70
    {  76,  91,  36 }, // 71
    {  16, 112, 112 }, // 72
    {  68,  88,   8 }, // 73
    {  16,  16,  28 }, // 74
    {  68,  88,   8 }, // 75
    {  16,  16,   8 }, // 76
    {  96,  88,  96 }, // 77
    {  16, 112, 116 }, // 78
    {  64,  64,  32 }, // 79
    { 100,  12, 112 }, // 80
    { 108, 108, 108 }, // 81
    {  12,  12,   0 }, // 82
    {  16,  16, 116 }, // 83
    {   0, 112,  32 }, // 84
    {  16, 112,  12 }, // 85
    {  16, 112,  12 }, // 86
    {  16,  16, 116 }, // 87
    {  16,  16, 116 }, // 88
    {  16,  16,  72 }, // 89
    { 111,   0,  56 }, // 90
    {  68,  68,   8 }, // 91
    {  16,  16,  28 }, // 92
    {  72,  88,  72 }, // 93
};

// RGB555 colours: 30 palettes of 4 laid end to end (lightest to darkest within a palette).
constexpr int PALETTE_COLOUR_COUNT = 120;
constexpr uint16_t PALETTE_COLOURS[PALETTE_COLOUR_COUNT] = {
    0x7FFF, 0x32BF, 0x00D0, 0x0000, // 0
    0x639F, 0x4279, 0x15B0, 0x04CB, // 1
    0x7FFF, 0x6E31, 0x454A, 0x0000, // 2
    0x7FFF, 0x1BEF, 0x0200, 0x0000, // 3
    0x7FFF, 0x421F, 0x1CF2, 0x0000, // 4
    0x7FFF, 0x5294, 0x294A, 0x0000, // 5
    0x7FFF, 0x03FF, 0x012F, 0x0000, // 6
    0x7FFF, 0x03EF, 0x01D6, 0x0000, // 7
    0x7FFF, 0x42B5, 0x3DC8, 0x0000, // 8
    0x7E74, 0x03FF, 0x0180, 0x0000, // 9
    0x67FF, 0x77AC, 0x1A13, 0x2D6B, // 10
    0x7ED6, 0x4BFF, 0x2175, 0x0000, // 11
    0x53FF, 0x4A5F, 0x7E52, 0x0000, // 12
    0x4FFF, 0x7ED2, 0x3A4C, 0x1CE0, // 13
    0x03ED, 0x7FFF, 0x255F, 0x0000, // 14
    0x036A, 0x021F, 0x03FF, 0x7FFF, // 15
    0x7FFF, 0x01DF, 0x0112, 0x0000, // 16
    0x231F, 0x035F, 0x00F2, 0x0009, // 17
    0x7FFF, 0x03EA, 0x011F, 0x0000, // 18
    0x299F, 0x001A, 0x000C, 0x0000, // 19
    0x7FFF, 0x027F, 0x001F, 0x0000, // 20
    0x7FFF, 0x03E0, 0x0206, 0x0120, // 21
    0x7FFF, 0x7EEB, 0x001F, 0x7C00, // 22
    0x7FFF, 0x3FFF, 0x7E00, 0x001F, // 23
    0x7FFF, 0x03FF, 0x001F, 0x0000, // 24
    0x03FF, 0x001F, 0x000C, 0x0000, // 25
    0x7FFF, 0x033F, 0x0193, 0x0000, // 26
    0x0000, 0x4200, 0x037F, 0x7FFF, // 27
    0x7FFF, 0x7E8C, 0x7C00, 0x0000, // 28
    0x7FFF, 0x1BEF, 0x6180, 0x0000, // 29
};
} // namespace compat_palette_data
