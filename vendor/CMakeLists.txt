# Third-party dependencies

# SDL3 - Build from source
set(SDL_SHARED OFF CACHE BOOL "Build SDL3 as a static library")
set(SDL_STATIC ON CACHE BOOL "Build SDL3 as a static library")
set(SDL_TEST_LIBRARY OFF CACHE BOOL "Don't build SDL test library")
add_subdirectory(SDL)

# ImGui - Build as a static library with SDL3 backend
add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_sdl3.cpp
    imgui/backends/imgui_impl_sdlrenderer3.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)

target_link_libraries(imgui PUBLIC SDL3::SDL3)
