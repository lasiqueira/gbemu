#include "compat_palettes.h"
#include "compat_palette_data.h"
#include <cstdint>

using namespace compat_palette_data;

namespace
{
    constexpr size_t HEADER_END = 0x150;

    bool is_nintendo_licensed(const std::vector<uint8_t>& rom)
    {
        if (rom[0x14B] == 0x33)
        {
            return rom[0x144] == '0' && rom[0x145] == '1';
        }
        return rom[0x14B] == 0x01;
    }

    int find_palette_id(uint8_t checksum, uint8_t fourth_letter)
    {
        int index = -1;
        for (int i = 0; i < 79; ++i)
        {
            if (kTitleChecksums[i] == checksum)
            {
                index = i;
                break;
            }
        }
        if (index < 0) return 0; // not in the table -> default
        if (index < kFirstAmbiguous) return index; // unambiguous: the index is the ID

        int column = index - kFirstAmbiguous;
        for (int pos = column; pos < 29; pos += kFourthLetterRowLength)
        {
            if (kFourthLetters[pos] == static_cast<char>(fourth_letter))
            {
                return kFirstAmbiguous + pos;
            }
        }
        return 0;
    }

    void copy_palette(uint16_t (&dst)[4], int offset)
    {
        for (int i = 0; i < 4; ++i)
        {
            dst[i] = kPaletteColours[offset + i];
        }
    }
}

CompatPalettes pick_compat_palettes(const std::vector<uint8_t>& rom)
{
    CompatPalettes result{};
    int id = 0;

    if (rom.size() >= HEADER_END && is_nintendo_licensed(rom))
    {
        uint8_t sum = 0;
        for (int i = 0x134; i <= 0x143; ++i) // 16 title bytes, low 8 bits only
        {
            sum += rom[i];
        }

        result.checksum = sum;
        id = find_palette_id(sum, rom[0x137]);
    }
    const PaletteSet& set = kPaletteSets[id];
    copy_palette(result.bg, set.bg);
    copy_palette(result.obj0, set.obj0);
    copy_palette(result.obj1, set.obj1);

    return result;
}