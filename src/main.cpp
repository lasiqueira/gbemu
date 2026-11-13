#include <vector>
#include <cstdint>
#include <print>
#include <format>
#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include "gameboy.h"
#include "disassembler.h"
#include <SDL3/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

constexpr int DISPLAY_SCALE = 4; // 4x scaling for 160x144 screen
constexpr Uint8 WINDOW_BACKGROUND_GREY = 20; // Colour behind the game screen and panels
#ifdef GBEMU_DEBUG
constexpr int DISASM_PANEL_WIDTH = 300; // Width of disassembly panel
constexpr int MEMORY_PANEL_WIDTH = 500; // Width of memory viewer panel
constexpr int CPU_PANEL_HEIGHT = 230; // Height of CPU state panel
constexpr float MODEL_COMBO_WIDTH = 80.0f; // Width of the model combo box in the options row
constexpr int MEMORY_BYTES_PER_ROW = 16; // Bytes shown per row in the memory viewer
constexpr uint16_t MEMORY_ROW_MASK = 0xFFF0; // Rounds an address down to the start of its row
constexpr ImU32 HIGHLIGHT_COLOUR = IM_COL32(255, 255, 0, 255); // Marks the byte/instruction at PC
#endif


struct WindowLayout
{
    int game_x = 0;
    int game_y = 0;
    int game_width = 0;
    int game_height = 0;
    int window_width = 0;
    int window_height = 0;
    int center_x = 0;
    int center_width = 0;
    int center_height = 0;
    // Debug-only — zero in release builds
    int disasm_width = 0;
    int cpu_height = 0;
    int memory_width = 0;
};


// globals
static GameBoy gameboy;
static SDL_Window* window = nullptr;
static SDL_Renderer* renderer = nullptr;
static SDL_Texture* screen_texture = nullptr;
static SDL_Gamepad* gamepad = nullptr;
static SDL_AudioStream* audio_stream = nullptr;
static SDL_AudioDeviceID audio_device = 0;
static WindowLayout window_layout;
static std::vector<uint8_t> loaded_rom;
static std::string loaded_rom_path;
static bool paused = false;

static WindowLayout compute_window_layout(int window_width, int window_height)
{
    WindowLayout layout;
    layout.window_width = window_width;
    layout.window_height = window_height;

    #ifdef GBEMU_DEBUG
    layout.disasm_width = DISASM_PANEL_WIDTH;
    layout.memory_width = MEMORY_PANEL_WIDTH;
    layout.cpu_height = CPU_PANEL_HEIGHT;
    layout.center_x = layout.disasm_width;
    layout.center_width = window_width - layout.disasm_width - layout.memory_width;
    layout.center_height = window_height - layout.cpu_height;
    #else
    layout.center_x = 0;
    layout.center_width = window_width;
    layout.center_height = window_height;
    #endif
    
    float gb_aspect = (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT;
    float center_aspect = (float) layout.center_width / (float) layout.center_height;

    int scaled_w, scaled_h;
    if (center_aspect > gb_aspect)
    {
        // wider than GB → pillarbox (bars on sides)
        scaled_h = layout.center_height;
        scaled_w = (int)(scaled_h * gb_aspect);
    }
    else
    {
        // taller → letterbox (bars on top/bottom)
        scaled_w = layout.center_width;
        scaled_h = (int)(scaled_w / gb_aspect);
    }
    layout.game_width  = scaled_w;
    layout.game_height = scaled_h;
    layout.game_x = layout.center_x + (layout.center_width  - scaled_w) / 2;
    layout.game_y = (layout.center_height - scaled_h) / 2;

    return layout;
}

static int init()
{   
        
    // Initialize SDL with video, gamepad, and audio support
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD | SDL_INIT_AUDIO))
    {
        std::println(std::cerr, "SDL_Init failed: {}", SDL_GetError());
        return 1;
    }

     // Open the first available gamepad
    gamepad = nullptr;
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
    #ifdef GBEMU_DEBUG
    int initial_window_width  = SCREEN_WIDTH * DISPLAY_SCALE + DISASM_PANEL_WIDTH + MEMORY_PANEL_WIDTH; // 1440
    int initial_window_height = SCREEN_HEIGHT * DISPLAY_SCALE + CPU_PANEL_HEIGHT;       // 776
    #else
    int initial_window_width  = SCREEN_WIDTH * DISPLAY_SCALE;
    int initial_window_height = SCREEN_HEIGHT * DISPLAY_SCALE;
    #endif
    // Compute window layout
    window_layout = compute_window_layout(initial_window_width, initial_window_height);

    window = SDL_CreateWindow(
        "Game Boy Emulator",
        window_layout.window_width, window_layout.window_height,
        SDL_WINDOW_RESIZABLE
    );
    

    
    if (!window)
    {
        std::println(std::cerr, "Failed to create window: {}", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    
    #ifdef GBEMU_DEBUG
    // Minimum: panels (800px) + at least 1x game width (160px) and 1x game height (144px) + cpu strip
    SDL_SetWindowMinimumSize(window, DISASM_PANEL_WIDTH + MEMORY_PANEL_WIDTH + SCREEN_WIDTH, CPU_PANEL_HEIGHT + SCREEN_HEIGHT);
    #else
    SDL_SetWindowMinimumSize(window, SCREEN_WIDTH, SCREEN_HEIGHT);
    #endif

    // Create renderer
    renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer)
    {
        std::println(std::cerr, "Failed to create renderer: {}", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    SDL_SetRenderVSync(renderer, 0); // Disable VSync, we'll control frame rate manually
    
    // Create texture for Game Boy screen
    screen_texture = SDL_CreateTexture(
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

    SDL_SetTextureScaleMode(screen_texture, SDL_SCALEMODE_NEAREST);

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
    
    //Initiate SDL Audio Stream
    SDL_AudioSpec spec{};
    spec.format = SDL_AUDIO_S16;
    spec.channels = 2;
    spec.freq = AUDIO_SAMPLE_RATE;

    // Open the default playback device
    audio_device = SDL_OpenAudioDevice(
        SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
        &spec
    );

    if (!audio_device)
    {
        std::println(std::cerr, "Failed to open audio: {}", SDL_GetError());
        return 1;
    }

    audio_stream = SDL_CreateAudioStream(&spec, &spec);

    if (!audio_stream)
    {
        std::println(std::cerr, "Failed to create audio stream: {}", SDL_GetError());
        return 1;
    }

    SDL_BindAudioStream(audio_device, audio_stream);
    SDL_ResumeAudioDevice(audio_device);

    return 0;

}

static void shutdown()
{

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
    
    SDL_DestroyAudioStream(audio_stream);
    SDL_CloseAudioDevice(audio_device);
    
    SDL_Quit();
}

static void update_window_title()
{
    const char* model = gameboy.model_is_auto ? "auto" : (gameboy.requested_model == Model::DMG ? "DMG" : "CGB");
    const char* running_as = !gameboy.cgb_mode ? "DMG" : (gameboy.dmg_compat ? "CGB (DMG compat)" : "CGB");
    std::string title = std::format("gbemu - {} [{} -> {}]", loaded_rom_path, model, running_as);
    SDL_SetWindowTitle(window, title.c_str());
}

static void reload_rom()
{
    gameboy.memory.save_battery(loaded_rom_path);
    bool color_correction = gameboy.ppu.color_correction;
    gameboy.load_rom(loaded_rom, loaded_rom_path);
    gameboy.ppu.color_correction = color_correction;
    gameboy.ppu.rebuild_palette_cache(gameboy.memory);
    SDL_ClearAudioStream(audio_stream);
    gameboy.running = true;
    update_window_title();
}

#ifdef GBEMU_DEBUG
// Emulator options shared by the hotkeys. Kept in one function so a future ImGui front-end can reuse it.
static void draw_emulator_options()
{
    if (ImGui::Checkbox("Colour correction", &gameboy.ppu.color_correction))
    {
        gameboy.ppu.rebuild_palette_cache(gameboy.memory);
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(MODEL_COMBO_WIDTH);
    const char* models[] = { "auto", "DMG", "CGB" };
    int current = gameboy.model_is_auto ? 0 : (gameboy.requested_model == Model::DMG ? 1 : 2);
    if (ImGui::Combo("Model", &current, models, IM_ARRAYSIZE(models)))
    {
        gameboy.model_is_auto = (current == 0);
        if (current != 0)
        {
            gameboy.requested_model = (current == 1) ? Model::DMG : Model::CGB;
        }
        reload_rom(); // the model only takes effect on a reload
    }
}
#endif

// Updates one joypad button (active low: 0 = pressed) and raises the joypad interrupt on a press.
// A mask of 0 means the input isn't mapped to any button and does nothing.
static void set_joypad_button(uint8_t button, bool pressed)
{
    uint8_t& joypad = gameboy.memory.joypad_state;
    uint8_t prev = joypad;
    if (pressed) joypad &= ~button; else joypad |= button;

    // Raise joypad interrupt if any button transitioned to pressed (1->0)
    if (prev & ~joypad)
    {
        gameboy.memory.io[IO_IF - ADDR_IO_START] |= INT_JOYPAD;
    }
}

// Handle keyboard input for Game Boy joypad
static void handle_input(SDL_Event& event)
{
    if (event.type != SDL_EVENT_KEY_DOWN && event.type != SDL_EVENT_KEY_UP)
        return;
    
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
    {
        switch (event.key.key)
        {
            case SDLK_F1: paused = !paused; return;
            case SDLK_F2: reload_rom(); return;
            case SDLK_F3:
                if (gameboy.model_is_auto)
                {
                    gameboy.model_is_auto = false;
                    gameboy.requested_model = Model::DMG;
                }
                else if (gameboy.requested_model == Model::DMG)
                {
                    gameboy.requested_model = Model::CGB;
                }
                else
                {
                    gameboy.model_is_auto = true;
                }
                reload_rom(); 
                return;
            case SDLK_F4:
                gameboy.ppu.color_correction = !gameboy.ppu.color_correction;
                gameboy.ppu.rebuild_palette_cache(gameboy.memory);
                return;
        }
    }


    bool pressed = (event.type == SDL_EVENT_KEY_DOWN);

    uint8_t button = 0;
    switch (event.key.key)
    {
        case SDLK_Z:      button = JOYPAD_A; break;
        case SDLK_X:      button = JOYPAD_B; break;
        case SDLK_RSHIFT:
        case SDLK_LSHIFT: button = JOYPAD_SELECT; break;
        case SDLK_RETURN: button = JOYPAD_START; break;
        case SDLK_RIGHT:  button = JOYPAD_RIGHT; break;
        case SDLK_LEFT:   button = JOYPAD_LEFT; break;
        case SDLK_UP:     button = JOYPAD_UP; break;
        case SDLK_DOWN:   button = JOYPAD_DOWN; break;
    }
    set_joypad_button(button, pressed);
}

static void handle_gamepad_input(SDL_Event& event)
{
    if (event.type != SDL_EVENT_GAMEPAD_BUTTON_DOWN && event.type != SDL_EVENT_GAMEPAD_BUTTON_UP)
        return;
    
    bool pressed = (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN);

    // Map gamepad buttons to Game Boy buttons
    uint8_t button = 0;
    switch (event.gbutton.button)
    {
        case SDL_GAMEPAD_BUTTON_SOUTH:      button = JOYPAD_A; break;      // Xbox: A, PS: X, Switch: B
        case SDL_GAMEPAD_BUTTON_EAST:       button = JOYPAD_B; break;      // Xbox: B, PS: Circle, Switch: A
        case SDL_GAMEPAD_BUTTON_BACK:       button = JOYPAD_SELECT; break; // Xbox: Back/View, PS: Share, Switch: Minus
        case SDL_GAMEPAD_BUTTON_START:      button = JOYPAD_START; break;  // Xbox: Start/Menu, PS: Options, Switch: Plus
        case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: button = JOYPAD_RIGHT; break;
        case SDL_GAMEPAD_BUTTON_DPAD_LEFT:  button = JOYPAD_LEFT; break;
        case SDL_GAMEPAD_BUTTON_DPAD_UP:    button = JOYPAD_UP; break;
        case SDL_GAMEPAD_BUTTON_DPAD_DOWN:  button = JOYPAD_DOWN; break;
    }
    set_joypad_button(button, pressed);
}

int main(int argc, char** argv)
{
    std::println("Game Boy Emulator");
    
    // Check command line arguments
    if (argc < 2)
    {
        std::println(std::cerr, "Usage: {} <rom_path> [--disassemble|-d] [--headless] [--model=auto|dmg|cgb]", argv[0]);
        return 1;
    }
    
    std::string const rom_path = argv[1];
    bool disassemble_mode = false;
    
    // Check for disassemble flag
    bool headless_mode = false;

    std::string model_arg = "cgb"; // default model is cgb

    for (int i = 2; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--disassemble" || arg == "-d")
            disassemble_mode = true;
        else if (arg == "--headless")
            headless_mode = true;
        else if (arg.rfind("--model=", 0) == 0)
            model_arg = arg.substr(8);
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

    // Determine the requested Game Boy model based on the command line argument
    gameboy.model_is_auto = (model_arg == "auto");
    if (!gameboy.model_is_auto)
    {
        if (model_arg == "dmg")
            gameboy.requested_model = Model::DMG;
        else if (model_arg == "cgb")
            gameboy.requested_model = Model::CGB;
        else
        {
            std::println(std::cerr, "Unknown model: {}", model_arg);
            return 1;
        }
    }

    loaded_rom = rom;
    loaded_rom_path = rom_path;
    // Initialize Game Boy and load ROM
    gameboy.load_rom(rom, rom_path);

    std::println("ROM loaded: {} bytes", rom.size());
    std::println("Model selected: {}", gameboy.model == Model::DMG ? "DMG" : "CGB");
    std::println("CGB mode: {}", gameboy.cgb_mode ? "Yes" : "No");
    std::println("DMG compatibility mode: {}", gameboy.dmg_compat ? "Yes" : "No");
    std::println("Starting emulation...");
    std::println("");

    // Headless mode: run without SDL, stop on pass/fail serial output or ext_ram result
    if (headless_mode)
    {
        gameboy.running = true;
        constexpr int HEADLESS_TIMEOUT_FRAMES = 60 * 120; // 120 seconds max
        // Signature written by blargg's init_text_out into ext_ram at offsets 1-3
        // $A001=$DE, $A002=$B0, $A003=$61 — indicates a RAM-output test has initialised
        auto ram_initialized = [&]() -> bool {
            const auto& r = gameboy.memory.ext_ram;
            return r.size() >= 8 && r[1] == 0xDE && r[2] == 0xB0 && r[3] == 0x61;
        };

        for (int frame = 0; frame < HEADLESS_TIMEOUT_FRAMES && gameboy.running; ++frame)
        {
            gameboy.step_frame();

            // Serial-output tests (cpu_instrs, instr_timing, mem_timing)
            const std::string& sout = gameboy.memory.serial_output;
            if (sout.find("Passed") != std::string::npos ||
                sout.find("passed") != std::string::npos ||
                sout.find("Failed") != std::string::npos ||
                sout.find("failed") != std::string::npos)
            {
                break;
            }

            // RAM-output tests (mem_timing-2, dmg_sound, oam_bug, halt_bug)
            // ext_ram[0] = final_result: $80 = in progress, 0 = pass, other = fail
            if (ram_initialized() && gameboy.memory.ext_ram[0] != 0x80)
            {
                // Print text output from ext_ram[4] onward (null-terminated string)
                const char* text = reinterpret_cast<const char*>(gameboy.memory.ext_ram.data() + 4);
                std::printf("%s\n", text);
                fflush(stdout);
                break;
            }
        }
        return 0;
    }
    
    if (init() != 0)
    {
        std::println(std::cerr, "Failed to initialize SDL or ImGui.");
        shutdown();
        return 1;
    }

    // After init(), before the main loop:
    // Pre-fill the audio queue with silence so the device has headroom against frame-time jitter.
    constexpr double AUDIO_TARGET_QUEUE_MS = 60.0;
    constexpr int AUDIO_BYTES_PER_SECOND = AUDIO_SAMPLE_RATE * 2 * static_cast<int>(sizeof(int16_t)); // stereo, 16-bit
    const int prefill_samples = static_cast<int>(AUDIO_BYTES_PER_SECOND * (AUDIO_TARGET_QUEUE_MS / 1000.0)) / static_cast<int>(sizeof(int16_t));
    std::vector<int16_t> silence(prefill_samples, 0);
    SDL_PutAudioStreamData(audio_stream, silence.data(), silence.size() * sizeof(int16_t));
    update_window_title();

    // Main loop
    bool quit = false;
    gameboy.running = true;
    
    // Frame timing, paced to the Game Boy's ~59.7 FPS
    auto next_frame = std::chrono::steady_clock::now();
    const auto frame_duration = std::chrono::duration<double>(1.0 / FRAME_RATE);
    while (!quit && gameboy.running)
    {        
        // Handle events
        SDL_Event event;
        int resize_w = 0, resize_h = 0;

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
            else if (event.type == SDL_EVENT_WINDOW_RESIZED)
            {
                resize_w = event.window.data1;
                resize_h = event.window.data2;
            }
            else if (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN || 
                     event.type == SDL_EVENT_GAMEPAD_BUTTON_UP)
            {
                handle_gamepad_input(event);
            }
            else
            {
                handle_input(event);
            }
        }
        
        if (resize_w > 0)
        {
            window_layout = compute_window_layout(resize_w, resize_h);
            resize_w = resize_h = 0;
            next_frame = std::chrono::steady_clock::now(); // reset after resize

        }
        // Run one frame of emulation (~70224 cycles)
        // A paused emulator neither runs nor queues audio; rendering and ImGui keep going
        if (!paused)
        {
            gameboy.step_frame();

            SDL_PutAudioStreamData(
                audio_stream,
                gameboy.apu.sample_buffer.data(),
                gameboy.apu.sample_buffer.size() * sizeof(int16_t)
            );

            gameboy.apu.sample_buffer.clear();
        }
        
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
        ImGui::SetNextWindowPos(ImVec2(window_layout.center_x, window_layout.center_height), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(window_layout.center_width, window_layout.cpu_height), ImGuiCond_Always);
        ImGui::Begin("CPU State", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        if (ImGui::BeginTable("cpu_row1", 4, ImGuiTableFlags_SizingStretchSame))
        {
            ImGui::TableNextColumn(); ImGui::Text("PC: 0x%04X", gameboy.cpu.pc);
            ImGui::TableNextColumn(); ImGui::Text("SP: 0x%04X", gameboy.cpu.sp);
            ImGui::TableNextColumn(); ImGui::Text("IME: %s", gameboy.cpu.ime ? "ON" : "OFF");
            ImGui::TableNextColumn(); ImGui::Text("Instructions: %llu", gameboy.cpu.instructions_executed);
            ImGui::EndTable();
        }
        
        ImGui::Separator();
        
        if (ImGui::BeginTable("cpu_regs", 2, ImGuiTableFlags_SizingStretchSame))
{
        ImGui::TableNextColumn(); ImGui::Text("AF: 0x%04X  (A: 0x%02X  F: 0x%02X)", gameboy.cpu.af.pair, gameboy.cpu.af.high, gameboy.cpu.af.low);
        ImGui::TableNextColumn(); ImGui::Text("BC: 0x%04X  (B: 0x%02X  C: 0x%02X)", gameboy.cpu.bc.pair, gameboy.cpu.bc.high, gameboy.cpu.bc.low);
        ImGui::TableNextColumn(); ImGui::Text("DE: 0x%04X  (D: 0x%02X  E: 0x%02X)", gameboy.cpu.de.pair, gameboy.cpu.de.high, gameboy.cpu.de.low);
        ImGui::TableNextColumn(); ImGui::Text("HL: 0x%04X  (H: 0x%02X  L: 0x%02X)", gameboy.cpu.hl.pair, gameboy.cpu.hl.high, gameboy.cpu.hl.low);
        ImGui::EndTable();
}
        
        ImGui::Separator();
        ImGui::Text("Flags:  Z: %d  N: %d  H: %d  C: %d",
            get_flag(gameboy.cpu.af.low, FLAG_ZERO),
            get_flag(gameboy.cpu.af.low, FLAG_SUBTRACT),
            get_flag(gameboy.cpu.af.low, FLAG_HALF_CARRY),
            get_flag(gameboy.cpu.af.low, FLAG_CARRY));
        ImGui::Separator();
        draw_emulator_options();
        ImGui::End();
        
        // Memory viewer - fixed to the right
        ImGui::SetNextWindowPos(ImVec2(window_layout.center_x + window_layout.center_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(window_layout.memory_width, window_layout.window_height), ImGuiCond_Always);
        ImGui::Begin("Memory Viewer", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Current Byte at PC: 0x%02X", gameboy.memory.read(gameboy.cpu.pc));
        ImGui::Separator();
        
        // Display memory around PC
        ImGui::BeginChild("MemoryScroll");
        float line_h = ImGui::GetTextLineHeightWithSpacing();
        int visible_rows = std::max(4, (int)(ImGui::GetContentRegionAvail().y / line_h));
        int half = visible_rows / 2;
        uint16_t start_addr = (gameboy.cpu.pc & MEMORY_ROW_MASK);
        for (int row = -half; row < half; row++)
        {
            uint16_t addr = start_addr + (row * MEMORY_BYTES_PER_ROW);
            ImGui::Text("%04X:", addr);
            ImGui::SameLine();
            
            for (int col = 0; col < MEMORY_BYTES_PER_ROW; col++)
            {
                uint16_t byte_addr = addr + col;
                uint8_t byte = gameboy.memory.read(byte_addr);
                
                // Highlight current PC
                if (byte_addr == gameboy.cpu.pc)
                {
                    ImVec2 pos = ImGui::GetCursorScreenPos();
                    ImVec2 size = ImGui::CalcTextSize("FF");
                    ImGui::GetWindowDrawList()->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), HIGHLIGHT_COLOUR);
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
                    ImGui::Text("%02X", byte);
                    ImGui::PopStyleColor();
                }
                else
                {
                    ImGui::Text("%02X", byte);
                }
                
                if (col < MEMORY_BYTES_PER_ROW - 1) ImGui::SameLine();
            }
        }
        ImGui::EndChild();
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(window_layout.disasm_width, window_layout.window_height), ImGuiCond_Always);
        ImGui::Begin("Disassembly", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::BeginChild("DisasmScroll");
        const auto& history = gameboy.cpu.pc_history;
        line_h = ImGui::GetTextLineHeightWithSpacing();
        int visible_lines = std::max(4, (int)(ImGui::GetContentRegionAvail().y / line_h));
        size_t start = history.size() > (size_t)visible_lines ? history.size() - visible_lines : 0;

        for (size_t i = start; i < history.size(); i++)
        {
            uint16_t addr = history[i];
            uint8_t bytes[3] = {
                gameboy.memory.read(addr),
                gameboy.memory.read(addr + 1),
                gameboy.memory.read(addr + 2)
            };

            auto instr = disassembler::decode_instruction(bytes, 3);
            std::string line = std::format("{:04X}  {} {}", addr, instr.mnemonic, instr.operands);
            bool is_current = (i == history.size() - 1);
            if (is_current)
            {
                ImVec2 pos = ImGui::GetCursorScreenPos();
                ImVec2 size = ImGui::CalcTextSize(line.c_str());
                ImGui::GetWindowDrawList()->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), HIGHLIGHT_COLOUR);
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
            }
            ImGui::TextUnformatted(line.c_str());
            if (is_current)
            {
                ImGui::PopStyleColor();
                ImGui::SetScrollHereY(1.0f);
            }
        }

        ImGui::EndChild();
        ImGui::End();
#endif
        
        // Render
        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, WINDOW_BACKGROUND_GREY, WINDOW_BACKGROUND_GREY, WINDOW_BACKGROUND_GREY, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);
        
        // Render Game Boy screen in top-left corner
        SDL_FRect game_rect = { (float)window_layout.game_x, (float)window_layout.game_y, (float)window_layout.game_width, (float)window_layout.game_height };
        SDL_RenderTexture(renderer, screen_texture, nullptr, &game_rect);
        
        // Render ImGui
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        
        SDL_RenderPresent(renderer);

        // Pace on the audio queue: when it runs low, emulate slightly faster, when it fills up, slightly
        // slower. A fixed timer drifts against the sound card and underruns the queue, which pops.
        double queued_ms = SDL_GetAudioStreamQueued(audio_stream) * 1000.0 / AUDIO_BYTES_PER_SECOND;
        double adjust_ms = std::clamp((queued_ms - AUDIO_TARGET_QUEUE_MS) * 0.05, -1.0, 1.0);
        next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            frame_duration + std::chrono::duration<double, std::milli>(adjust_ms));

        auto now = std::chrono::steady_clock::now();
        if (next_frame < now)
            next_frame = now;

        std::this_thread::sleep_until(next_frame);

    }

     gameboy.memory.save_battery(rom_path); // Save battery RAM if applicable
    
    shutdown();
    
    return 0;
}
