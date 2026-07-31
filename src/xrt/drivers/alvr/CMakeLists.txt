# Copyright 2024, DCS Digital / Scott Bowler
# SPDX-License-Identifier: BSL-1.0

# ALVR driver — Monado ↔ ALVR streaming bridge for ARM64 (aarch64) Linux.
# Blender OpenXR → Monado → alvr_target (NVENC) → Vision Pro

cmake_minimum_required(VERSION 3.13)

# ── Find dependencies ──────────────────────────────────────────────────────────

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavutil)

# alvr_server_core — built via: cd /home/scott/alvr-src && cargo xtask build-server-lib --release
set(ALVR_SERVER_CORE_DIR "/home/scott/alvr-src/build/alvr_server_core"
    CACHE PATH "Path to built alvr_server_core library")

if(NOT EXISTS "${ALVR_SERVER_CORE_DIR}/alvr_server_core.h")
  message(FATAL_ERROR
    "alvr_server_core not found at ${ALVR_SERVER_CORE_DIR}. "
    "Run: cd /home/scott/alvr-src && cargo xtask build-server-lib --release")
endif()

# Find the compiled .a or .so
find_library(ALVR_SERVER_CORE_LIB
  NAMES alvr_server_core
  PATHS "${ALVR_SERVER_CORE_DIR}"
        "/home/scott/alvr-src/target/release"
  NO_DEFAULT_PATH)

if(NOT ALVR_SERVER_CORE_LIB)
  message(FATAL_ERROR "Could not find libAlvr_server_core. Did the Rust build complete?")
endif()

message(STATUS "ALVR server core: ${ALVR_SERVER_CORE_LIB}")

# ── Driver library ─────────────────────────────────────────────────────────────

add_library(drv_alvr STATIC
  alvr_prober.c
  alvr_hmd.cpp
  alvr_target.cpp
  alvr_interface.h
)

target_include_directories(drv_alvr PRIVATE
  "${ALVR_SERVER_CORE_DIR}"
  "${FFMPEG_INCLUDE_DIRS}"
  "/usr/local/cuda-13.0/targets/sbsa-linux/include"
  # Monado generated headers (vk_helpers_h_ext.h.inc etc.)
  "${CMAKE_BINARY_DIR}/src/xrt/auxiliary/vk"
  "${CMAKE_BINARY_DIR}/src/xrt/include"
  # Monado compositor internals
  "${CMAKE_SOURCE_DIR}/src/xrt/compositor"
  "${CMAKE_SOURCE_DIR}/src/xrt/auxiliary"
  "${CMAKE_SOURCE_DIR}/src/external"
)

target_link_libraries(drv_alvr PRIVATE
  xrt-interfaces
  aux_util
  aux_math
  comp_main
  comp_util
  "${ALVR_SERVER_CORE_LIB}"
  ${FFMPEG_LIBRARIES}
  pthread
  dl
)

target_compile_options(drv_alvr PRIVATE
  -Wall -Wextra -Wno-unused-parameter
)

# C++17 for structured bindings / designated initialisers
set_property(TARGET drv_alvr PROPERTY CXX_STANDARD 17)
