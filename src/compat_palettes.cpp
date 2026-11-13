#include "compat_palettes.h"
#include "compat_palette_data.h"
#include "constants.h"
#include <cstddef>
#include <cstdint>
#include <vector>

using namespace compat_palette_data;

static bool is_nintendo_licensed(const std::vector<uint8_t>& rom)
{
    if (rom[ROM_HEADER_OLD_LICENSEE] == LICENSEE_USE_NEW_FIELD)
    {
        return rom[ROM_HEADER_NEW_LICENSEE] == '0' && rom[ROM_HEADER_NEW_LICENSEE + 1] == '1';
    }
    return rom[ROM_HEADER_OLD_LICENSEE] == LICENSEE_NINTENDO_OLD;
}

static int find_palette_id(uint8_t checksum, uint8_t fourth_letter)
{
    int index = -1;
    for (int i = 0; i < TITLE_CHECKSUM_COUNT; ++i)
    {
        if (TITLE_CHECKSUMS[i] == checksum)
        {
            index = i;
            break;
        }
    }
    if (index < 0) return 0; // not in the table -> default
    if (index < FIRST_AMBIGUOUS_CHECKSUM) return index; // unambiguous: the index is the ID

    int column = index - FIRST_AMBIGUOUS_CHECKSUM;
    for (int pos = column; pos < FOURTH_LETTER_COUNT; pos += FOURTH_LETTER_ROW_LENGTH)
    {
        if (FOURTH_LETTERS[pos] == static_cast<char>(fourth_letter))
        {
            return FIRST_AMBIGUOUS_CHECKSUM + pos;
        }
    }
    return 0;
}

static bool needs_logo_tilemap(uint8_t checksum)
{
    for (uint8_t logo_checksum : LOGO_TILEMAP_CHECKSUMS)
    {
        if (checksum == logo_checksum) return true;
    }
    return false;
}

static void copy_palette(uint16_t (&dst)[COLOURS_PER_PALETTE], int offset)
{
    for (int i = 0; i < COLOURS_PER_PALETTE; ++i)
    {
        dst[i] = PALETTE_COLOURS[offset + i];
    }
}

CompatPalettes pick_compat_palettes(const std::vector<uint8_t>& rom)
{
    CompatPalettes result{};
    int id = 0;

    if (rom.size() >= ROM_HEADER_END && is_nintendo_licensed(rom))
    {
        uint8_t sum = 0;
        for (int i = 0; i < ROM_HEADER_TITLE_LENGTH; ++i) // low 8 bits only
        {
            sum += rom[ROM_HEADER_TITLE + i];
        }

        result.checksum = sum;
        result.logo_tilemap = needs_logo_tilemap(sum);
        id = find_palette_id(sum, rom[ROM_HEADER_FOURTH_LETTER]);
    }
    const PaletteSet& set = PALETTE_SETS[id];
    copy_palette(result.bg, set.bg);
    copy_palette(result.obj0, set.obj0);
    copy_palette(result.obj1, set.obj1);

    return result;
}