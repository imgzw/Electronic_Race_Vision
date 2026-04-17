# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

To disable Rockchip RGA acceleration:
```bash
cmake -DENABLE_RGA=OFF ..
```

The binary is output to `build/car`.

## Run

```bash
# Camera + serial + HTTP preview
./build/car --camera-device=/dev/video0 --http-mjpeg=8080 --headless /dev/ttyS2

# Offline debug with image or video
./build/car test.png
./build/car video.mp4

# Quick start script
./start.bash
```

## Architecture

Everything lives in a single file: `car.cpp`.

**Pipeline (single frame loop):**
1. Capture frame (camera / file) → optional RGA resize to `1280×720`
2. ROI crop → grayscale → binary threshold
3. Scan `NUM_STRIPS` horizontal strips, compute white-pixel centroid per strip
4. Outlier rejection → EMA smoothing → polynomial fit → extract heading angle at `LOOKAHEAD_RATIO`
5. Send heading over serial port (unit: 0.01°, range ±90°)
6. Optionally push annotated frame via HTTP MJPEG (`MjpegServer`) or GStreamer UDP (`RemoteStreamWorker`)

**Key types:**
- `RuntimeOptions` — parsed CLI flags
- `TrackResult` — per-frame output: `heading_deg`, `centroids`, validity flags
- `TrackingState` — inter-frame EMA state and lost-line counter
- `MjpegServer` — single-client HTTP MJPEG server (raw TCP, no external HTTP lib)
- `RemoteStreamWorker` — background thread that owns `MjpegServer` and optional GStreamer pipeline

**RGA acceleration** is conditionally compiled via `HAVE_RGA`. If librga is absent, all resize falls back to `cv::resize`. The fallback path is transparent to the rest of the code.

**All tunable parameters** (strip count, EMA alpha, ROI ratios, serial scale, etc.) are `const` globals at the top of `car.cpp`.
