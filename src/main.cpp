#include <vector>
#include <cstdint>
#include <print>
#include <filesystem>
#include <iostream>
#include <string>
#include "gameboy.h"
#include "disassembler.h"
#include <SDL3/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

// Handle keyboard input for Game Boy joypad
void handle_input(SDL_Event& event, GameBoy& gameboy)
{
    if (event.type != SDL_EVENT_KEY_DOWN && event.type != SDL_EVENT_KEY_UP)
        return;
    
    bool pressed = (event.type == SDL_EVENT_KEY_DOWN);
    uint8_t& joypad = gameboy.memory.joypad_state;
    
    // Joypad state: bit 0 = pressed, 1 = released
    // Bits 0-3: A, B, Select, Start
    // Bits 4-7: Right, Left, Up, Down
    switch (event.key.key)
    {
        case SDLK_Z:      // A button
            if (pressed) joypad &= ~0x01; else joypad |= 0x01;
            break;
        case SDLK_X:      // B button
            if (pressed) joypad &= ~0x02; else joypad |= 0x02;
            break;
        case SDLK_RSHIFT: // Select
        case SDLK_LSHIFT:
            if (pressed) joypad &= ~0x04; else joypad |= 0x04;
            break;
        case SDLK_RETURN: // Start
            if (pressed) joypad &= ~0x08; else joypad |= 0x08;
            break;
        case SDLK_RIGHT:  // Right
            if (pressed) joypad &= ~0x10; else joypad |= 0x10;
            break;
        case SDLK_LEFT:   // Left
            if (pressed) joypad &= ~0x20; else joypad |= 0x20;
            break;
        case SDLK_UP:     // Up
            if (pressed) joypad &= ~0x40; else joypad |= 0x40;
            break;
        case SDLK_DOWN:   // Down
            if (pressed) joypad &= ~0x80; else joypad |= 0x80;
            break;
    }
}

void handle_gamepad_input(SDL_Event& event, GameBoy& gameboy)
{
    if (event.type != SDL_EVENT_GAMEPAD_BUTTON_DOWN && event.type != SDL_EVENT_GAMEPAD_BUTTON_UP)
        return;
    
    bool pressed = (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN);
    uint8_t& joypad = gameboy.memory.joypad_state;
    
    // Map gamepad buttons to Game Boy buttons
    // Joypad state: bit 0 = pressed (active low)
    // Bits 0-3: A, B, Select, Start
    // Bits 4-7: Right, Left, Up, Down
    switch (event.gbutton.button)
    {
        case SDL_GAMEPAD_BUTTON_SOUTH:  // A button (Xbox: A, PS: X, Switch: B)
            if (pressed) joypad &= ~0x01; else joypad |= 0x01;
            break;
        case SDL_GAMEPAD_BUTTON_EAST:   // B button (Xbox: B, PS: Circle, Switch: A)
            if (pressed) joypad &= ~0x02; else joypad |= 0x02;
            break;
        case SDL_GAMEPAD_BUTTON_BACK:   // Select (Xbox: Back/View, PS: Share, Switch: Minus)
            if (pressed) joypad &= ~0x04; else joypad |= 0x04;
            break;
        case SDL_GAMEPAD_BUTTON_START:  // Start (Xbox: Start/Menu, PS: Options, Switch: Plus)
            if (pressed) joypad &= ~0x08; else joypad |= 0x08;
            break;
        case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:
            if (pressed) joypad &= ~0x10; else joypad |= 0x10;
            break;
        case SDL_GAMEPAD_BUTTON_DPAD_LEFT:
            if (pressed) joypad &= ~0x20; else joypad |= 0x20;
            break;
        case SDL_GAMEPAD_BUTTON_DPAD_UP:
            if (pressed) joypad &= ~0x40; else joypad |= 0x40;
            break;
        case SDL_GAMEPAD_BUTTON_DPAD_DOWN:
            if (pressed) joypad &= ~0x80; else joypad |= 0x80;
            break;
    }
}

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
    
    // Initialize SDL with video and gamepad support
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
    {
        std::println(std::cerr, "SDL_Init failed: {}", SDL_GetError());
        return 1;
    }
    
    // Open the first available gamepad
    SDL_Gamepad* gamepad = nullptr;
    int num_joysticks = 0;
    SDL_JoystickID* joysticks = SDL_GetGamepads(&num_joysticks);
    if (num_joysticks > 0)
    {
        gamepad = SDL_OpenGamepad(joysticks[0]);
        if (gamepad)
        {
            std::println("Gamepad connected: {}", SDL_GetGamepadName(gamepad));
        }
    }
    SDL_free(joysticks);
    
    // Create window with space for debug panels
    const int scale = 4; // 4x scaling for 160x144 screen
    const int game_width = SCREEN_WIDTH * scale;   // 640
    const int game_height = SCREEN_HEIGHT * scale; // 576
    
#ifdef GBEMU_DEBUG
    const int cpu_height = 200;
    const int memory_width = 500;
    const int window_width = game_width + memory_width;   // 1140
    const int window_height = game_height + cpu_height;   // 776
#else
    const int window_width = game_width;
    const int window_height = game_height;
#endif
    
    SDL_Window* window = SDL_CreateWindow(
        "Game Boy Emulator",
        window_width, window_height,
        0
    );
    
    if (!window)
    {
        std::println(std::cerr, "Failed to create window: {}", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    
    // Create renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer)
    {
        std::println(std::cerr, "Failed to create renderer: {}", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    SDL_SetRenderVSync(renderer, 0); // Disable VSync, we'll control frame rate manually
    
    // Create texture for Game Boy screen
    SDL_Texture* screen_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH,
        SCREEN_HEIGHT
    );
    
    if (!screen_texture)
    {
        std::println(std::cerr, "Failed to create texture: {}", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    // Setup ImGui style and font scale
    ImGui::StyleColorsDark();
    io.FontGlobalScale = 1.3f;  // Make font 30% bigger
    
    // Setup Platform/Renderer backends
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);
    
    // Main loop
    bool quit = false;
    gameboy.running = true;
    
    // Frame timing for 60 FPS
    const double target_frame_time = 1000.0 / 59.7; // Game Boy runs at ~59.7 Hz
    Uint64 last_frame_time = SDL_GetTicks();
    
    while (!quit && gameboy.running)
    {
        Uint64 frame_start = SDL_GetTicks();
        
        // Handle events
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);
            
            if (event.type == SDL_EVENT_QUIT)
            {
                quit = true;
            }
            else if (event.type == SDL_EVENT_GAMEPAD_ADDED)
            {
                // Open the newly connected gamepad if we don't have one
                if (!gamepad)
                {
                    gamepad = SDL_OpenGamepad(event.gdevice.which);
                    if (gamepad)
                    {
                        std::println("Gamepad connected: {}", SDL_GetGamepadName(gamepad));
                    }
                }
            }
            else if (event.type == SDL_EVENT_GAMEPAD_REMOVED)
            {
                // Close the disconnected gamepad
                if (gamepad && SDL_GetGamepadID(gamepad) == event.gdevice.which)
                {
                    std::println("Gamepad disconnected");
                    SDL_CloseGamepad(gamepad);
                    gamepad = nullptr;
                }
            }
            else if (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN || 
                     event.type == SDL_EVENT_GAMEPAD_BUTTON_UP)
            {
                handle_gamepad_input(event, gameboy);
            }
            else
            {
                handle_input(event, gameboy);
            }
        }
        
        // Run one frame of emulation (~70224 cycles)
        gameboy.step_frame();
        
        // Update screen texture if frame is ready
        if (gameboy.ppu.frame_ready)
        {
            SDL_UpdateTexture(
                screen_texture,
                nullptr,
                gameboy.ppu.rgba_buffer.data(),
                SCREEN_WIDTH * 4
            );
            gameboy.ppu.frame_ready = false;
        }
        
        // Start ImGui frame
        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        
#ifdef GBEMU_DEBUG
        // CPU State window - fixed below game screen
        ImGui::SetNextWindowPos(ImVec2(0, game_height), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(game_width, cpu_height), ImGuiCond_Always);
        ImGui::Begin("CPU State", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("PC: 0x%04X", gameboy.cpu.pc);
        ImGui::SameLine(150);
        ImGui::Text("SP: 0x%04X", gameboy.cpu.sp);
        ImGui::SameLine(300);
        ImGui::Text("IME: %s", gameboy.cpu.ime ? "ON" : "OFF");
        ImGui::SameLine(450);
        ImGui::Text("Instructions: %llu", gameboy.cpu.instructions_executed);
        
        ImGui::Separator();
        
        ImGui::Text("AF: 0x%04X  (A: 0x%02X  F: 0x%02X)", 
            gameboy.cpu.af.pair, gameboy.cpu.af.high, gameboy.cpu.af.low);
        ImGui::SameLine(300);
        ImGui::Text("BC: 0x%04X  (B: 0x%02X  C: 0x%02X)", 
            gameboy.cpu.bc.pair, gameboy.cpu.bc.high, gameboy.cpu.bc.low);
        
        ImGui::Text("DE: 0x%04X  (D: 0x%02X  E: 0x%02X)", 
            gameboy.cpu.de.pair, gameboy.cpu.de.high, gameboy.cpu.de.low);
        ImGui::SameLine(300);
        ImGui::Text("HL: 0x%04X  (H: 0x%02X  L: 0x%02X)", 
            gameboy.cpu.hl.pair, gameboy.cpu.hl.high, gameboy.cpu.hl.low);
        
        ImGui::Separator();
        ImGui::Text("Flags:  Z: %d  N: %d  H: %d  C: %d",
            cpu::get_flag(gameboy.cpu.af.low, cpu::FLAG_ZERO),
            cpu::get_flag(gameboy.cpu.af.low, cpu::FLAG_SUBTRACT),
            cpu::get_flag(gameboy.cpu.af.low, cpu::FLAG_HALF_CARRY),
            cpu::get_flag(gameboy.cpu.af.low, cpu::FLAG_CARRY));
        ImGui::End();
        
        // Memory viewer - fixed to the right
        ImGui::SetNextWindowPos(ImVec2(game_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(memory_width, window_height), ImGuiCond_Always);
        ImGui::Begin("Memory Viewer", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Current Byte at PC: 0x%02X", gameboy.memory.read(gameboy.cpu.pc));
        ImGui::Separator();
        
        // Display memory around PC
        ImGui::BeginChild("MemoryScroll");
        uint16_t start_addr = (gameboy.cpu.pc & 0xFFF0); // Align to 16 bytes
        for (int row = -8; row < 32; row++)
        {
            uint16_t addr = start_addr + (row * 16);
            ImGui::Text("%04X:", addr);
            ImGui::SameLine();
            
            for (int col = 0; col < 16; col++)
            {
                uint16_t byte_addr = addr + col;
                uint8_t byte = gameboy.memory.read(byte_addr);
                
                // Highlight current PC
                if (byte_addr == gameboy.cpu.pc)
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
                    ImGui::Text("%02X", byte);
                    ImGui::PopStyleColor();
                }
                else
                {
                    ImGui::Text("%02X", byte);
                }
                
                if (col < 15) ImGui::SameLine();
            }
        }
        ImGui::EndChild();
        ImGui::End();
#endif
        
        // Render
        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);
        
        // Render Game Boy screen in top-left corner
        SDL_FRect game_rect = { 0, 0, (float)game_width, (float)game_height };
        SDL_RenderTexture(renderer, screen_texture, nullptr, &game_rect);
        
        // Render ImGui
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        
        SDL_RenderPresent(renderer);
        
        // Frame rate limiting to ~60 FPS
        Uint64 frame_end = SDL_GetTicks();
        double elapsed = frame_end - frame_start;
        if (elapsed < target_frame_time)
        {
            SDL_Delay((Uint32)(target_frame_time - elapsed));
        }
    }
    
    // Cleanup
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    
    SDL_DestroyTexture(screen_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    
    if (gamepad)
    {
        SDL_CloseGamepad(gamepad);
    }
    
    SDL_Quit();
    
    return 0;
}
