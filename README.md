# Monado ALVR Driver — Apple Vision Pro Streaming

A Monado driver that streams any OpenXR application (including Blender) live to Apple Vision Pro via ALVR, built natively for ARM64 Linux.

## What it does

```
OpenXR app (Blender, etc.)
    ↓ xrBeginFrame / xrEndFrame
Monado OpenXR runtime
    ↓ comp_target (NVENC capture)
FFmpeg NVENC (h264/hevc/av1, GPU-native on ARM64/Blackwell)
    ↓ alvr_send_video_nal()
alvr_server_core (Rust networking library)
    ↓ Wi-Fi (9943/9944)
Apple Vision Pro (ALVR app — App Store, free)
```

## Status

- ✅ `xrt_device` (HMD tracking): pose data received from Vision Pro via `alvr_get_device_motion()`
- ✅ `comp_target` (compositor backend): Vulkan readback → NVENC → `alvr_send_video_nal()`
- ✅ Builds natively on aarch64 (NVIDIA DGX Spark / GB10)
- ✅ Device detected: `Apple Vision Pro (ALVR)` with 2 views
- ⚠️  Compositor needs GPU-backed headless Vulkan (VK_ICD_FILENAMES) — segfaults with llvmpipe
- ⚠️  Dynamic encoder params not yet wired to `alvr_get_dynamic_encoder_params()`

## Files

```
src/xrt/drivers/alvr/
  alvr_interface.h     — public interface (device + target + factory)
  alvr_prober.c        — xrt_auto_prober (always detects ALVR HMD)
  alvr_hmd.cpp         — xrt_device (pose tracking from Vision Pro)
  alvr_target.cpp      — comp_target (frame capture, NVENC, streaming)
  CMakeLists.txt       — build, links alvr_server_core + FFmpeg

Modified Monado files (diffs):
  src/xrt/drivers/CMakeLists.txt                 — add_subdirectory(alvr)
  src/xrt/targets/common/target_lists.c          — register auto-prober
  src/xrt/targets/common/CMakeLists.txt          — link drv_alvr
  src/xrt/compositor/main/comp_compositor.c      — inject factory via weak symbol
```

## Build

### Prerequisites

1. **ALVR server library** (ARM64):
   ```bash
   # Requires two upstream fixes first:
   # https://github.com/alvr-org/ALVR/pull/3345
   git clone --recurse-submodules https://github.com/alvr-org/ALVR.git
   # Apply ARM64 fix: s/[0i8; 1024]/[0u8; 1024]/ in alvr/server_openvr/src/lib.rs
   # Build OpenVR for ARM64: https://github.com/ValveSoftware/openvr/pull/1924
   cargo xtask prepare-deps --platform linux
   cargo xtask build-server-lib --release
   ```

2. **Monado source** + dependencies:
   ```bash
   sudo apt install libeigen3-dev glslang-tools libvulkan-dev \
     libwayland-dev libhidapi-dev ninja-build
   git clone --depth 1 https://gitlab.freedesktop.org/monado/monado.git
   ```

3. **Copy driver files** into Monado source tree:
   ```bash
   cp -r src/xrt/drivers/alvr  monado/src/xrt/drivers/
   # Apply the diffs to the three modified files
   ```

4. **Build**:
   ```bash
   cmake -B build -G Ninja \
     -DALVR_SERVER_CORE_DIR=/path/to/alvr/build/alvr_server_core \
     -DCUDA_INCLUDE_DIR=/usr/local/cuda-13.0/targets/sbsa-linux/include
   ninja monado-service
   ```

## Usage

```bash
# 1. Start ALVR streamer (provides network + session.json)
DISPLAY=:99 /path/to/alvr-arm64/bin/alvr_dashboard &

# 2. Start our Monado service (replaces stock monado-service)
XRT_NO_STDIN=1 DISPLAY=:99 \
  XR_RUNTIME_JSON=/path/to/our/monado-service/openxr_monado.json \
  ./build/src/xrt/targets/service/monado-service &

# 3. Open ALVR app on Vision Pro → Connect

# 4. Launch Blender in VR mode
XR_RUNTIME_JSON=... blender
# N panel → VR Scene Inspection → Start VR Session
```

## Hardware tested

| Hardware | OS | GPU | NVENC |
|---|---|---|---|
| NVIDIA DGX Spark (GB10 / Blackwell) | Ubuntu 24.04 | aarch64 | h264/hevc/av1 ✅ |

## Related upstream work

- [ALVR ARM64 build fix](https://github.com/alvr-org/ALVR/pull/3345) — char signedness + OpenVR docs
- [OpenVR ARM64 prebuilt](https://github.com/ValveSoftware/openvr/pull/1924) — native library

## Licence

BSL-1.0 (matching Monado)
