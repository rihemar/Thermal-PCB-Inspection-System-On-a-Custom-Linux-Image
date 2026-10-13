# Thermal Inspector

RGB + thermal fusion for PCB inspection. One executable, `main`, built from
`src/main.cpp` on top of the `thermal_core` library (cameras, thermal sources,
processing).

Pipeline in live mode:

```
RGB camera ──┐
             ├─► PCB detection ─► crop RGB to the board
thermal ─────┘   homography   ─► re-sample thermal onto the same area
                       │
                       ▼
             Joint bilateral upsampling
                       │
                       ▼
             Hotspot detection (threshold)
                       │
                       ▼
             Live heatmap GUI
```

---

## 1. Building

### Requirements

- CMake ≥ 3.16
- A C++17 compiler (g++ / clang++)
- OpenCV 4 (`libopencv-dev` on Ubuntu / Raspberry Pi OS)
- Raspberry Pi only: `bcm2835` library (see below) for the MLX90641 thermal sensor

### Project layout expected by `CMakeLists.txt`

```
Stage_LAB619/
├── CMakeLists.txt
├── include/
│   ├── camera/        IRGBCamera.hpp, WebcamCamera.hpp, IPWebcamCamera.hpp, RecordedRGB.hpp
│   ├── thermalCam/    ITCamera.hpp, ThermalCamera.hpp, fakeThermalCamera.hpp,
│   │                  RecordedThermal.hpp, generateFakeHeatmapFromImage.hpp
│   ├── processing/    HotspotDetector.hpp, JointBilateralUpsample.hpp,
│   │                  ChessboardCalibration.hpp, PCBDetector.hpp, Registration.hpp
│   └── drivers/MLX90641/
└── src/
    ├── main.cpp
    ├── camera/ … thermalCam/ … processing/ … drivers/MLX90641/ …
```

### Ubuntu / PC

```bash
cmake -B build -DDEVICE=UBUNTU
cmake --build build -j
```

Produces `build/main`. The real MLX90641 sensor is **not** compiled in this
configuration; `main -T real` prints an error and exits.

### Raspberry Pi

One-time: install the `bcm2835` library (downloads, builds and `sudo make install`s it):

```bash
cmake -B build -DDEVICE=RASPBERRY_PI
cmake --build build --target bcm
```

Then build the project:

```bash
cmake --build build -j
```

`DEVICE=RASPBERRY_PI` adds the MLX90641 driver sources, `ThermalCamera.cpp`, links
`bcm2835` and defines `DEVICE_RASPBERRY_PI`, which enables the `-T real` option.

### Switching device

`DEVICE` is cached by CMake. To switch, reconfigure with a fresh build directory:

```bash
rm -rf build
cmake -B build -DDEVICE=RASPBERRY_PI
cmake --build build -j
```
--- 

## 2. Running `main`

```
./build/main [<mode>] [options]
./build/main --help
```

### Modes

Given as the first positional argument or with `-m / --mode`. Without a mode the
interactive menu is shown.

| Mode           | Description |
|----------------|-------------|
| `menu`         | Interactive menu (default). `1` / `2` / `3` pick a mode, `Q` quits. |
| `live`         | Live heatmap: PCB crop, registration, joint bilateral upsampling, hotspot detection. |
| `auto-calib`   | Automated calibration: looks for a chessboard in every frame and saves the homography as soon as one is found. |
| `manual-calib` | Manual calibration: click matching points in the RGB and thermal images, then `ENTER`. |

When a mode is given on the command line, leaving it (`M` / `ESC`, or a finished
calibration) **quits** the program; from the menu it returns to the menu.

### Source options

| Option | Values | Default | Meaning |
|--------|--------|---------|---------|
| `-r, --rgb <src>` | `webcam`, `ipwebcam`, `recorded` | `webcam` | RGB source: `WebcamCamera`, `IPWebcamCamera`, `RecordedRGB` |
| `-T, --thermal <src>` | `fake-image`, `fake`, `recorded`, `real` | `fake-image` | Thermal source: `generateFakeHeatmapFromImage` (derived from the RGB frame, 16×12), `fakeThermalCamera` (random 32×24), `RecordedThermal`, `ThermalCamera` (MLX90641, Raspberry Pi build only) |
| `-d, --device <idx>` | integer | `0` | Webcam device index (`webcam` only) |
| `-u, --url <url>` | URL | class default | IP Webcam stream URL (`ipwebcam` only), e.g. `http://192.168.1.20:8080/video` |
| `-W, --width <px>` | integer | `1280` | Requested RGB capture width |
| `-H, --height <px>` | integer | `720` | Requested RGB capture height |
| `-a, --addr <hex>` | `0x33`… | `0x33` | MLX90641 I2C slave address (`real` only) |

### Display options

| Option | Default | Meaning |
|--------|---------|---------|
| `-D, --display <WxH>` | `1024x640` | Maximum on-screen size of the window. The rendered picture is scaled down to fit (processing still runs at camera resolution). Use your screen size, e.g. `-D 800x480` on a Pi touchscreen. |
| `-f, --fullscreen` | off | Start in fullscreen. `F` toggles it at runtime. |

### Processing options

| Option | Default | Meaning |
|--------|---------|---------|
| `-t, --threshold <C>` | `84` | Initial hotspot threshold in °C (slider range 30–120) |
| `-b, --board <CxR>` | `9x6` | Chessboard inner corners for automated calibration |
| `-o, --homography <file>` | `rgb_to_thermal_homography.yml` | Homography file: written by both calibration modes, read by live mode |
| `--no-crop` | crop on | Don't crop to the detected PCB in live mode |
| `--no-register` | register on | Don't apply the homography in live mode (assume same field of view) |

### Examples

```bash
# Menu, PC webcam, fake thermal derived from the RGB image (works without hardware)
./build/main

# Straight into live mode
./build/main live

# Phone camera over IP Webcam, fake thermal
./build/main live -r ipwebcam -u http://192.168.100.158:8080/video

# Raspberry Pi: USB camera + real MLX90641
./build/main live -r webcam -d 0 -T real -a 0x33

# Automated chessboard calibration with a 7x5 board, custom output file
./build/main auto-calib -b 7x5 -o calib_lab.yml

# Manual calibration on a small touchscreen, fullscreen
./build/main manual-calib -D 800x480 -f

# Recorded data for deterministic tests, no crop
./build/main live -r recorded -T recorded --no-crop

# Lower initial threshold
./build/main live -t 60
```

---

## 3. Keys and controls

### Every mode

| Key | Action |
|-----|--------|
| `Q` | Quit |
| `M` / `ESC` | Back to menu (or quit when launched in a mode) |
| `F` | Toggle fullscreen |

### Live heatmap

| Control | Action |
|---------|--------|
| Slider (mouse) | Hotspot threshold (°C) |
| `CALIBRATE` button | Jump to automated calibration |
| `C` | Toggle PCB crop on/off |

Status line (top-left): `PCB: found / not found`, `Crop: on / off`,
`Registration: on / no homography file / off`.

The fused heatmap is drawn at the PCB's position in the full frame with a yellow
outline around the crop.

### Automated calibration

Hold the chessboard in view of both cameras. As soon as it is detected in both,
the RGB→thermal homography is computed, the reprojection error is shown, the file
is saved and the mode exits after 1.5 s.

### Manual calibration

RGB on the left, thermal on the right (zoomed so each sensor pixel is a block).
The pane that expects the next click has a yellow border; points are numbered and
equal numbers form a pair.

| Key | Action |
|-----|--------|
| Click | Add a point (RGB first, then the same physical point in the thermal image) |
| `SPACE` | Freeze / unfreeze the frames while clicking |
| `U` | Undo the last point |
| `R` | Clear all points |
| `ENTER` | Compute the homography (≥ 4 pairs, least squares) and save it |

Tip: spread the points over the whole board (corners, connectors, large
components) — four nearly collinear points give a useless homography.

---

## 4. Homography file

Both calibration modes write the same file (default `rgb_to_thermal_homography.yml`)
through `ChessboardCalibration::SaveHomography`. It maps **RGB pixel → thermal
pixel**. Live mode reloads it every time it is entered, so the workflow is:

```
main            # menu
  2 or 3        # calibrate
  (back)        # file saved
  1             # live mode now uses the new homography
```

or, from the command line, run `main auto-calib` once and then `main live`.