#include <vector>
#include <cstdint>
#include <print>
#include <filesystem>
#include <iostream>
#include <string>
#include "gameboy.h"
#include "disassembler.h"

int main(int argc, char** argv)
{
    std::println("Game Boy Emulator");
    
    // Check command line arguments
    if (argc < 2)
    {
        std::println(std::cerr, "Usage: {} <rom_path> [--disassemble|-d]", argv[0]);
        return 1;
    }
    
    const std::string rom_path = argv[1];
    bool disassemble_mode = false;
    
    // Check for disassemble flag
    if (argc >= 3)
    {
        std::string arg2 = argv[2];
        if (arg2 == "--disassemble" || arg2 == "-d")
        {
            disassemble_mode = true;
        }
    }
    
    // Check if file exists
    if (!std::filesystem::exists(rom_path))
    {
        std::println(std::cerr, "ROM file not found: {}", rom_path);
        return 1;
    }
    
    // Get file size using std::filesystem
    size_t file_size = std::filesystem::file_size(rom_path);
    
    // Open ROM file with fopen (faster than ifstream)
    FILE* file = fopen(rom_path.c_str(), "rb");
    if (!file)
    {
        std::println(std::cerr, "Failed to open ROM file.");
        return 1;
    }
    
    // Read ROM into vector
    std::vector<uint8_t> rom(file_size);
    size_t bytes_read = fread(rom.data(), 1, file_size, file);
    fclose(file);
    
    if (bytes_read != file_size)
    {
        std::println(std::cerr, "Failed to read ROM file completely.");
        return 1;
    }
    
    // If disassemble mode, print disassembly and exit
    if (disassemble_mode)
    {
        disassembler::disassemble_rom(rom, 0, rom.size());
        return 0;
    }
    
    // Initialize Game Boy and load ROM
    GameBoy gameboy;
    gameboy.load_rom(rom);
    
    std::println("ROM loaded: {} bytes", rom.size());
    std::println("Starting emulation...");
    std::println("");
    
    // Run the emulator
    gameboy.run();
    
    return 0;
}
