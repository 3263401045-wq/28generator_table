cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# ---------------------------------------------------------------------------
# 没指定交叉工具链时，强制使用工程自带的 arm-none-eabi 工具链。
# 否则 CLion 默认的 MinGW 会用主机 gcc 去编译 startup_stm32f103xb.s，
# 报 “unknown pseudo-op: .syntax / no such instruction: cpsid i”。
# 想换工具链时，命令行 -DCMAKE_TOOLCHAIN_FILE=... 显式指定即可。
# ---------------------------------------------------------------------------
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "STM32 cross toolchain" FORCE)
endif()

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 28generator_table)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# 确认真的在用 ARM 交叉编译器，否则给出比汇编报错更明确的提示
if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi")
    message(FATAL_ERROR
        "当前编译器是 ${CMAKE_C_COMPILER}，不是 arm-none-eabi-gcc。\n"
        "CLion 请在 CMake 配置里选择 “Debug - Debug” 预设（生成目录 build/Debug），\n"
        "或删除 cmake-build-debug 目录后执行 Reset CMake Cache 重新配置。")
endif()

# 旧缓存若残留主机 MinGW 的标准库（-loleaut32 等），链接会失败；
# CMake 切换编译器时不会自动清理，这里兜底。
if(CMAKE_C_STANDARD_LIBRARIES MATCHES "oleaut32")
    unset(CMAKE_C_STANDARD_LIBRARIES CACHE)
    unset(CMAKE_C_STANDARD_LIBRARIES)
    unset(CMAKE_CXX_STANDARD_LIBRARIES CACHE)
    unset(CMAKE_CXX_STANDARD_LIBRARIES)
    message(STATUS "Cleared stale Windows standard libraries from CMake cache")
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/stepper.c
    Core/Src/cli.c
    Core/Src/app.c
    Core/Src/OLED.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 生成 bin/hex，方便 STM32CubeProgrammer / ST-Link Utility 烧录
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    BYPRODUCTS ${CMAKE_PROJECT_NAME}.bin ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin / .hex"
)
