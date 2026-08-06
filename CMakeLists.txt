cmake_minimum_required(VERSION 3.20)
project(gbemu CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Option to enable debug features (instruction tracing, counting, etc.)
option(GBEMU_DEBUG "Enable debug features (instruction tracing, counting)" OFF)

if(GBEMU_DEBUG)
    add_compile_definitions(GBEMU_DEBUG)
    message(STATUS "GBEMU_DEBUG enabled - instruction tracing and counting active")
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


if(MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()
# Add march=native flag for all platforms
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND MSVC)
    add_compile_options(-march=native $<$<NOT:$<CONFIG:Debug>>:-O3> $<$<NOT:$<CONFIG:Debug>>:-flto=thin>)
    add_link_options($<$<NOT:$<CONFIG:Debug>>:-flto=thin>)
elseif(MSVC)
    # /GL enables whole-program optimization; /LTCG enables LTO at link time; both non-Debug only
    add_compile_options(/arch:AVX2 $<$<NOT:$<CONFIG:Debug>>:/O2> $<$<NOT:$<CONFIG:Debug>>:/GL>)
    add_link_options($<$<NOT:$<CONFIG:Debug>>:/LTCG>)
else()
    add_compile_options(-march=native -O3)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-flto=thin)
        add_link_options(-flto=thin)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        add_compile_options(-flto)
        add_link_options(-flto)
    endif()
endif()

# Enable optimizations for Release builds
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add third-party dependencies
add_subdirectory(vendor)

# Enable testing
enable_testing()

# Core library with implementation files
add_library(gbemu_core 
    src/cpu.cpp
    src/memory.cpp
    src/gameboy.cpp
    src/disassembler.cpp
    src/ppu.cpp
    src/apu.cpp
)
target_include_directories(gbemu_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(gbemu_core PUBLIC SDL3::SDL3 imgui)

# Main executable
add_executable(gbemu src/main.cpp)
target_link_libraries(gbemu gbemu_core)

# Test executable
add_executable(test_disasm src/test_disasm.cpp)
target_link_libraries(test_disasm gbemu_core)

# Register test
add_test(NAME disassembler_test COMMAND test_disasm)
set_tests_properties(disassembler_test PROPERTIES FAIL_REGULAR_EXPRESSION "Error")

# Platform-specific settings
if(APPLE)
    message(STATUS "Building for macOS with -march=native")
elseif(WIN32)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND MSVC)
        message(STATUS "Building for Windows with clang-cl using -march=native")
    else()
        message(STATUS "Building for Windows with MSVC using /arch:AVX2")
    endif()
    if(MSVC)
        target_compile_options(gbemu PRIVATE /W4)
        target_compile_options(test_disasm PRIVATE /W4)
    endif()
else()
    message(STATUS "Building for Linux with -march=native")
endif()

# Print build info
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Run tests with: ctest --output-on-failure")
