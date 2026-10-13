# Embedded Edge-Based Multi-Spectral PCB Fault Detection System

## 1. Project Overview

### Objective

Build a production-oriented embedded vision system that combines:

- A high-resolution RGB camera for PCB detection, component/trace spatial information, and visual guidance.
- A low-resolution thermal sensor/camera for temperature measurements.
- Geometric calibration and registration between the two sensors.
- RGB-guided Joint Bilateral Upsampling (JBU) to reconstruct an edge-aware high-resolution thermal field.
- Thermal anomaly/hotspot detection.
- Component-aware fault localization.
- A complete C++ embedded application architecture.
- Hardware-independent simulation and replay modes for development without physical hardware.
- Embedded Linux deployment using Yocto.
- Automatic system startup using systemd.
- Fault tolerance for camera disconnection, I2C errors, memory pressure, application crashes, and sensor timeouts.
- Automated tests and performance benchmarking.

The project should be developed first on a PC using a normal webcam and a simulated thermal camera, then deployed to a Raspberry Pi + real thermal sensor when hardware becomes available.

---

# 2. Main Engineering Goal

The project should NOT be treated as:

> "An OpenCV application that puts a thermal image over an RGB image."

It should be treated as:

> **An embedded edge-computing system integrating heterogeneous sensors, hardware abstraction, real-time acquisition, calibration, sensor fusion, edge-aware thermal reconstruction, fault detection, and robust Embedded Linux deployment.**

The final system should demonstrate:

- C++
- Object-oriented architecture
- CMake
- OpenCV
- Linux
- Embedded Linux
- I2C
- Camera acquisition
- Hardware abstraction
- Sensor fusion
- Computer vision
- Multithreading
- Memory/resource management
- Error handling
- systemd
- Yocto/BitBake
- Automated testing
- Performance optimization
- Networking/telemetry

---

# 3. Target Architecture

```text
                           EMBEDDED LINUX
                     Raspberry Pi / SBC + Yocto
                                  |
                           systemd service
                                  |
                                  v
                     +-------------------------+
                     | Thermal PCB Inspection  |
                     |       Application       |
                     +------------+------------+
                                  |
                    +-------------+-------------+
                    |                           |
                    v                           v
              RGB Acquisition             Thermal Acquisition
                    |                           |
              Camera HAL                    Thermal HAL
                    |                           |
                 V4L2 /                       I2C
               libcamera                 MLX90640/etc.
                    |                           |
                    +-------------+-------------+
                                  |
                                  v
                         Timestamp / Sync
                                  |
                                  v
                         Frame Registration
                                  |
                                  v
                    RGB-Guided Thermal Upsampling
                                  |
                                  v
                         Thermal Processing
                                  |
                    +-------------+-------------+
                    |                           |
                    v                           v
               Hotspot/Fault             Visualization
                Detection                 / Heatmap
                    |                           |
                    +-------------+-------------+
                                  |
                                  v
                         Fault Localization
                                  |
                                  v
                       Telemetry / Interface
                         MQTT / REST / UI
```

---

# 4. Development Modes

The application must support three modes.

## 4.1 Simulation Mode

Runs entirely on the development PC.

```text
PC Webcam
    +
Simulated Thermal Camera
    +
Mock I2C
    +
C++ Processing
```

Used for:

- Algorithm development
- Unit testing
- Fault injection
- Performance testing
- Development without hardware

---

## 4.2 Replay Mode

Uses recorded RGB and thermal data.

```text
Recorded RGB Frames
        +
Recorded Thermal Frames
        |
        v
   Same Interfaces
        |
        v
   Same Processing
```

Used for:

- Reproducible testing
- Regression testing
- Debugging
- Benchmarking
- Comparing algorithm versions

---

## 4.3 Hardware Mode

Final deployment.

```text
Real RGB Camera
      +
Real MLX90640 / thermal sensor
      +
Linux I2C
      +
Camera subsystem
      |
      v
Same application core
```

The processing code should not need to know whether data came from simulation or real hardware.

---

# 5. Repository Structure

Recommended structure:

```text
thermal-pcb-inspection/
|
+-- README.md
+-- CMakeLists.txt
+-- LICENSE
|
+-- docs/
|   +-- architecture.md
|   +-- requirements.md
|   +-- calibration.md
|   +-- thermal-fusion.md
|   +-- fault-tolerance.md
|   +-- deployment.md
|
+-- include/
|   +-- interfaces/
|   |   +-- IRGBCamera.hpp
|   |   +-- IThermalCamera.hpp
|   |   +-- II2CBus.hpp
|   |   +-- IFrameSource.hpp
|   |
|   +-- camera/
|   |   +-- WebcamCamera.hpp
|   |   +-- V4L2Camera.hpp
|   |
|   +-- thermal/
|   |   +-- FakeThermalCamera.hpp
|   |   +-- MLX90640Camera.hpp
|   |
|   +-- processing/
|   |   +-- PCBDetector.hpp
|   |   +-- HotspotDetector.hpp
|   |   +-- EdgeDetector.hpp
|   |   +-- ThermalUpsampler.hpp
|   |   +-- ThermalColorMapper.hpp
|   |
|   +-- calibration/
|   |   +-- CameraCalibration.hpp
|   |   +-- CheckerboardDetector.hpp
|   |   +-- Registration.hpp
|   |
|   +-- synchronization/
|   |   +-- FrameSynchronizer.hpp
|   |   +-- Timestamp.hpp
|   |
|   +-- communication/
|   |   +-- MQTTClient.hpp
|   |
|   +-- system/
|       +-- Logger.hpp
|       +-- HealthMonitor.hpp
|       +-- Watchdog.hpp
|
+-- src/
|   +-- camera/
|   +-- thermal/
|   +-- drivers/
|   +-- processing/
|   +-- calibration/
|   +-- synchronization/
|   +-- communication/
|   +-- system/
|   +-- application/
|
+-- simulation/
|   +-- FakeThermalCamera.cpp
|   +-- MockI2CBus.cpp
|   +-- FaultInjector.cpp
|   +-- ThermalScene.cpp
|
+-- replay/
|   +-- RecordedRGB.cpp
|   +-- RecordedThermal.cpp
|
+-- tests/
|   +-- test_hotspot.cpp
|   +-- test_registration.cpp
|   +-- test_thermal_upsampling.cpp
|   +-- test_synchronization.cpp
|   +-- test_i2c.cpp
|   +-- test_fault_recovery.cpp
|
+-- datasets/
|   +-- calibration/
|   +-- rgb/
|   +-- thermal/
|   +-- replay/
|
+-- config/
|   +-- system.yaml
|   +-- calibration.yaml
|
+-- systemd/
|   +-- thermal-inspection.service
|
+-- yocto/
|   +-- meta-pcb-inspection/
|
└-- scripts/
    +-- calibrate.py
    +-- record.py
    +-- benchmark.py
```

---

# 6. Phase 1 — C++ Foundation

## Goals

Create:

- CMake project
- C++17 or newer
- OpenCV
- GoogleTest
- Logging
- Basic application entry point

Initial application:

```text
Application started
Configuration loaded
Simulation mode
System ready
```

CMake should support selectable build modes:

```text
SIMULATION=ON
REPLAY=ON/OFF
HARDWARE=ON/OFF
TESTS=ON/OFF
```

Example:

```bash
cmake -S . -B build -DSIMULATION=ON
cmake --build build
```

---

# 7. Phase 2 — Hardware Abstraction

The most important architectural principle is:

> The application must depend on interfaces, not hardware implementations.

## RGB interface

```cpp
class IRGBCamera {
public:
    virtual bool initialize() = 0;
    virtual bool captureFrame(RGBFrame& frame) = 0;
    virtual bool isConnected() const = 0;
    virtual void shutdown() = 0;

    virtual ~IRGBCamera() = default;
};
```

Implementations:

```text
IRGBCamera
 |
 +-- WebcamCamera
 |
 +-- V4L2Camera
 |
 +-- RecordedRGB
```

---

## Thermal interface

```cpp
class IThermalCamera {
public:
    virtual bool initialize() = 0;
    virtual bool captureFrame(ThermalFrame& frame) = 0;
    virtual bool isConnected() const = 0;
    virtual void shutdown() = 0;

    virtual ~IThermalCamera() = default;
};
```

Implementations:

```text
IThermalCamera
 |
 +-- FakeThermalCamera
 |
 +-- RecordedThermal
 |
 +-- MLX90640Camera
```

---

# 8. Thermal Data Model

The thermal sensor should be represented by actual temperature values, not by an already colorized image.

Example:

```cpp
struct ThermalFrame {
    std::vector<float> temperatures;
    int width;
    int height;
    uint64_t timestamp;
};
```

For an MLX90640:

```text
width  = 32
height = 24
```

The raw data remains:

```text
30.2 30.5 31.0 ...
29.8 31.1 35.4 ...
...
```

Do NOT permanently convert the temperature data into RGB colors.

---

# 9. Phase 3 — Fake Thermal Camera

This is the first major simulation component.

## Objective

Simulate a low-resolution thermal sensor with:

- Ambient temperature
- Temperature noise
- Hot components
- Multiple hotspots
- Heating/cooling over time
- Dead pixels
- Sensor timeout
- Invalid frames
- Temperature drift

---

## 9.1 Basic thermal scene

Start with:

```text
Thermal resolution = 32 x 24
Ambient = 25-30 C
Noise = approximately +/-0.5 C
```

Create synthetic hotspots:

```text
Hotspot 1:
center = (15, 10)
temperature = 80 C
radius = 3 pixels
```

Use a Gaussian heat source:

T(x,y) = T_ambient + A * exp(
    -((x-x0)^2 + (y-y0)^2) / (2*sigma^2)
)

Where:

- T_ambient = background temperature
- A = hotspot temperature increase
- (x0,y0) = hotspot center
- sigma = spatial spread

This produces a realistic smooth heat source rather than a rectangular artificial block.

---

# 10. Simulate Real PCB Thermal Behavior

Create several classes of regions:

```text
PCB background
Traces
Passive components
ICs
Connectors
Hot components
```

Example:

```text
PCB = 28 C

Resistor = 34 C

Normal IC = 38 C

Overheating IC = 75 C
```

Add spatial Gaussian heating around components.

Eventually simulate:

```text
temperature(t)
```

so that a component can gradually heat:

```text
30 C
35 C
42 C
50 C
60 C
70 C
```

This lets you test time-based fault detection.

---

# 11. Thermal Sensor Noise

Add:

```text
Gaussian noise
dead pixels
random outliers
small bias/drift
```

For example:

T_measured = T_real + noise + bias

where:

```text
noise ~ N(0, sigma_noise)
```

This tests robustness.

---

# 12. Mock I2C

Create an interface:

```cpp
class II2CBus {
public:
    virtual bool write(
        uint8_t address,
        const uint8_t* data,
        size_t size) = 0;

    virtual bool read(
        uint8_t address,
        uint8_t* buffer,
        size_t size) = 0;

    virtual ~II2CBus() = default;
};
```

Implement:

```text
II2CBus
 |
 +-- LinuxI2CBus
 |
 +-- MockI2CBus
```

The real implementation eventually uses:

```text
/dev/i2c-X
```

The mock implementation uses memory/register simulation.

---

# 13. Phase 4 — RGB Camera Acquisition

Start with the PC webcam.

Use OpenCV initially.

Later the hardware implementation can use Linux camera APIs such as V4L2/libcamera depending on the selected camera stack.

RGBFrame should contain:

```cpp
struct RGBFrame {
    cv::Mat image;
    uint64_t timestamp;
};
```

The camera module is responsible for:

- Initialization
- Capture
- Timestamping
- Resolution
- FPS
- Disconnect detection
- Shutdown

---

# 14. Phase 5 — Frame Synchronization

The RGB and thermal sensors have different frame rates.

Example:

```text
RGB      = 30 FPS
Thermal  = 8 FPS
```

Therefore never simply use:

```text
RGB frame N + Thermal frame N
```

Instead timestamp every frame.

Use:

```cpp
uint64_t timestamp;
```

Maintain buffers:

```text
RGB queue
Thermal queue
```

For each RGB frame, select the closest thermal frame within an allowed time difference.

Example:

```text
RGB:
1000 ms

Thermal:
998 ms
```

Difference:

```text
2 ms
```

Accept.

If:

```text
RGB = 1000 ms
Thermal = 850 ms
```

Difference:

```text
150 ms
```

Reject or mark synchronization as poor.

---

# 15. Phase 6 — PCB Detection

The RGB image provides spatial structure.

Initial pipeline:

```text
RGB
 |
v
Grayscale
 |
v
Gaussian blur
 |
v
Canny edges
 |
v
Morphological cleanup
 |
v
Contour detection
 |
v
Polygon approximation
 |
v
PCB quadrilateral
```

Use:

```cpp
cv::Canny()
cv::findContours()
cv::approxPolyDP()
```

Output:

```cpp
struct PCBGeometry {
    std::array<cv::Point2f,4> corners;
    cv::Mat mask;
    bool detected;
};
```

---

# 16. Phase 7 — Automatic Checkerboard Calibration

Calibration must determine the spatial relationship between the RGB and thermal sensors.

## 16.1 Calibration target

Use a checkerboard visible to both sensors.

Important consideration:

A normal printed black/white checkerboard may not provide a sufficiently strong thermal pattern for the thermal sensor.

The target should therefore be designed so that:

- RGB camera clearly sees checkerboard corners.
- Thermal camera can distinguish alternating thermal regions.

Possible approach:

```text
Thermally contrasting checkerboard
```

For example, alternating materials/surfaces with different emissivity or controlled heating.

---

# 17. RGB Checkerboard Detection

Capture multiple calibration views.

Use OpenCV:

```cpp
cv::findChessboardCorners()
```

Then refine:

```cpp
cv::cornerSubPix()
```

For each image:

```text
RGB image
   |
   v
Find checkerboard
   |
   v
Corner coordinates
   |
   v
Sub-pixel refinement
```

Store:

```text
imagePointsRGB
objectPoints
```

---

# 18. Thermal Checkerboard Detection

The thermal image may be low-resolution and noisy.

Possible pipeline:

```text
Thermal temperature matrix
       |
       v
Normalize
       |
       v
Denoise
       |
       v
Temperature/gradient image
       |
       v
Checkerboard detection
```

Depending on the physical calibration target, detect the thermal checkerboard corners using:

- thresholding
- gradients
- contours
- corner detection
- known checkerboard geometry

The thermal image should first be upsampled only for corner localization if needed, while the original temperature measurements remain the source data.

---

# 19. Calibration Strategy

There are two related calibration problems.

## A. Intrinsic calibration

Determine each camera's own parameters:

```text
fx
fy
cx
cy
distortion coefficients
```

For the RGB camera, use:

```cpp
cv::calibrateCamera()
```

The thermal camera can be calibrated if its optical model and sufficient calibration data are available.

---

## B. Extrinsic / RGB-Thermal registration

Because the PCB is approximately planar, a homography is an excellent first deployment model.

Collect corresponding points:

```text
thermal checkerboard corners
        ↕
RGB checkerboard corners
```

Calculate:

```cpp
cv::findHomography()
```

Prefer:

```cpp
cv::RANSAC
```

to reject bad corner correspondences.

Mathematically:

```text
[x_rgb]
[y_rgb] = H [x_thermal]
[  1  ]     [y_thermal]
            [   1   ]
```

After homogeneous transformation:

```text
x' = ...
y' = ...

x_rgb = x'/w'
y_rgb = y'/w'
```

---

# 20. Automatic Calibration Procedure

Implement a calibration application:

```text
START CALIBRATION
       |
       v
Capture RGB + Thermal
       |
       v
Detect checkerboard
       |
       +----> RGB corners
       |
       +----> Thermal corners
       |
       v
Validate detection
       |
       v
Collect correspondence
       |
       v
Repeat for multiple poses
       |
       v
Estimate calibration parameters
       |
       v
Calculate reprojection error
       |
       v
If error acceptable:
       save calibration
Else:
       request more samples
```

Store calibration:

```yaml
homography:
  - [h11, h12, h13]
  - [h21, h22, h23]
  - [h31, h32, h33]

reprojection_error: X

rgb:
  width: 1280
  height: 720

thermal:
  width: 32
  height: 24
```

---

# 21. Calibration Quality Metrics

Never just calculate a homography and assume it is correct.

Measure:

## Reprojection error

For every known corresponding point:

```text
actual RGB point
        vs
projected RGB point
```

Compute:

e_i = ||p_actual - p_projected||

Then:

RMSE = sqrt(
    sum(e_i^2) / N
)

Report:

```text
Mean error
RMSE
Maximum error
```

For a PCB inspection system, also measure error in physical units if the PCB dimensions are known.

---

# 22. Phase 8 — Thermal Registration

After calibration:

```cpp
cv::warpPerspective()
```

maps the thermal field into RGB coordinates.

Important:

Do not convert the raw thermal temperatures into a color image before geometric processing.

Preferred order:

```text
Raw temperatures
      |
      v
Calibration / registration
      |
      v
Temperature field in RGB coordinates
      |
      v
Edge-aware upsampling
      |
      v
Color mapping
```

---

# 23. Phase 9 — Joint Bilateral Upsampling

This is the key algorithm for the desired FLIR-like visualization.

Problem:

```text
Thermal = low resolution
RGB     = high resolution
```

Normal resizing causes thermal information to bleed across component boundaries.

Instead use the RGB image as a spatial guide.

For target high-resolution pixel p:

T(p) =
    [sum_q w(p,q) T(q)]
    /
    [sum_q w(p,q)]

where:

- p = target RGB pixel
- q = nearby thermal sample
- T(q) = measured temperature
- w(p,q) = influence of thermal sample q on p

---

# 24. Spatial Weight

Use:

w_spatial(p,q) =
exp(
    -||p-q||^2 / (2 sigma_s^2)
)

Purpose:

> Nearby thermal pixels should have more influence than distant pixels.

sigma_s controls spatial influence.

Small sigma_s:

```text
sharper
less smoothing
more noise
```

Large sigma_s:

```text
smoother
more interpolation
more thermal bleeding
```

---

# 25. RGB Range Weight

Use:

w_RGB(p,q) =
exp(
    -||I(p)-I(q)||^2 / (2 sigma_r^2)
)

where I is the RGB guide image or a feature derived from it.

Purpose:

> Pixels with similar appearance should influence each other more than pixels separated by strong visual boundaries.

At a component boundary:

```text
RGB difference = large
```

Therefore:

```text
w_RGB ≈ 0
```

and thermal information does not easily cross the boundary.

---

# 26. Combined Joint Bilateral Weight

Use:

w(p,q) =
w_spatial(p,q) * w_RGB(p,q)

Therefore:

T(p) =
[
sum_q
w_spatial(p,q)
w_RGB(p,q)
T(q)
]
/
[
sum_q
w_spatial(p,q)
w_RGB(p,q)
]

Interpretation:

A thermal sample must be:

1. spatially close
2. visually compatible with the RGB region

to have a strong influence.

---

# 27. Why This Is Appropriate for PCB Inspection

PCBs contain strong spatial boundaries:

```text
IC edge
component edge
pad edge
trace edge
PCB border
```

RGB captures these boundaries at high resolution.

Thermal captures:

```text
temperature
thermal gradients
hotspots
```

The two sensors therefore provide complementary information.

The RGB image should guide **where thermal information should propagate**, while the thermal sensor remains the source of temperature measurements.

---

# 28. Important Scientific Limitation

Do not claim that JBU creates new real thermal measurements.

If the sensor is:

```text
32 x 24
```

the actual temperature information remains limited by that sensor.

RGB guidance can:

- preserve boundaries
- improve spatial localization
- reduce visual bleeding
- produce a sharper thermal visualization

It cannot magically measure the exact temperature of a tiny component that was never resolved by the thermal sensor.

This distinction should be documented in the project.

---

# 29. PCB-Aware Edge Guidance

For better PCB results, calculate RGB edges.

Example:

```cpp
cv::Sobel()
cv::Canny()
```

Then optionally introduce an edge penalty:

w(p,q) =
w_spatial *
w_RGB *
w_edge

For example:

w_edge =
exp(
    -E(p,q)^2 / (2 sigma_e^2)
)

where E represents an edge/boundary difference.

Strong PCB edge:

```text
E large
w_edge small
```

Therefore interpolation is discouraged across the edge.

---

# 30. Edge-Aware Filtering Options

Implement in stages.

### Version 1

Bilinear/cubic interpolation.

Purpose:

- Establish baseline.

### Version 2

Bilateral filtering.

Purpose:

- Preserve thermal edges.

### Version 3

Joint Bilateral Upsampling.

Purpose:

- RGB-guided thermal reconstruction.

### Version 4

Guided filter / edge-aware refinement.

Purpose:

- Compare against JBU.

Benchmark all versions.

---

# 31. Thermal Gradient Enhancement

Calculate:

Gx = dT/dx

Gy = dT/dy

Temperature gradient magnitude:

|grad T| = sqrt(Gx^2 + Gy^2)

Use:

```cpp
cv::Sobel()
```

This identifies locations where temperature changes rapidly.

High gradient regions can be emphasized for visualization.

Important:

Gradient enhancement is a visualization technique. It must not alter the underlying measured temperature used for fault decisions.

---

# 32. Thermal Color Mapping

After reconstructing the high-resolution temperature field:

```text
Temperature field
       |
       v
Normalize temperature
       |
       v
0..255
       |
       v
Thermal colormap
```

For example:

```text
cold
  |
blue
  |
cyan
  |
green
  |
yellow
  |
orange
  |
red
  |
white/pink
  |
hot
```

A custom colormap can be implemented instead of blindly using `COLORMAP_JET`.

---

# 33. Temperature Normalization

Avoid relying only on automatic min/max for a production system.

Possible modes:

### Fixed range

Example:

```text
20 C -> blue
50 C -> red/white
```

Advantage:

- Comparability between frames.

### Dynamic range

Use current frame min/max.

Advantage:

- Strong visual contrast.

Disadvantage:

- Same temperature can appear as different colors in different frames.

Recommended:

Use a configurable fixed engineering range for fault analysis, with optional dynamic mode for exploration.

---

# 34. Final Composition

After alignment and reconstruction:

```text
RGB image
    +
high-resolution thermal color image
```

A simple baseline:

```cpp
cv::addWeighted(
    rgb,
    0.6,
    thermalColor,
    0.4,
    0,
    result
);
```

But the final system should eventually support:

- constant alpha
- temperature-dependent alpha
- hotspot emphasis
- edge-preserving composition
- component masks

---

# 35. Component-Aware Visualization

Detect or segment component regions from RGB.

Create masks:

```text
PCB mask
component mask
trace mask
background mask
```

Then use them to control visualization.

For example:

```text
Normal PCB area:
moderate thermal visibility

Component area:
strong thermal visibility

Detected hotspot:
very strong thermal visibility + marker
```

This makes the result more useful for inspection than a generic heatmap.

---

# 36. Fault Detection

Do not define faults only as:

```text
temperature > X
```

Use multiple criteria.

Possible features:

```text
Absolute temperature
Temperature above local PCB background
Temperature gradient
Hotspot area
Heating rate
Component type
Distance to component boundary
```

For example:

DeltaT = T_component - T_local_background

A component can be suspicious if:

```text
T_component > absolute_threshold
```

OR:

```text
DeltaT > relative_threshold
```

OR:

```text
dT/dt > heating_rate_threshold
```

This is more robust than one global threshold.

---

# 37. Hotspot Detection Pipeline

```text
Thermal field
     |
     v
Denoising
     |
     v
Temperature threshold
     |
     v
Binary hotspot mask
     |
     v
Morphological cleanup
     |
     v
Connected components
     |
     v
Remove tiny/noisy regions
     |
     v
Calculate:
    area
    centroid
    max temperature
    mean temperature
    delta temperature
     |
     v
Fault candidate
```

---

# 38. Component-Level Fault Localization

The final objective is not merely:

```text
Hotspot at pixel (X,Y)
```

It should be:

```text
Fault candidate:
Component region
Location
Maximum temperature
Average temperature
Temperature above local baseline
Confidence
```

Example:

```json
{
  "component_id": "unknown_or_detected",
  "x": 850,
  "y": 430,
  "max_temperature": 78.4,
  "local_background": 31.2,
  "delta_temperature": 47.2,
  "severity": "critical"
}
```

---

# 39. Multithreaded Acquisition Architecture

Do not make one giant sequential loop.

Use producer/consumer architecture.

```text
RGB Capture Thread
       |
       v
RGB Ring Buffer
       |
       |
       +----------------+
                        |
                        v
                 Synchronizer
                        ^
                        |
       +----------------+
       |
Thermal Capture Thread
       |
       v
Thermal Ring Buffer

                        |
                        v
                 Processing Thread
                        |
                        v
                   Result Queue
                        |
              +---------+---------+
              |                   |
              v                   v
          Display             Telemetry
```

Use:

- `std::thread`
- `std::mutex`
- `std::condition_variable`
- bounded queues
- timestamps

---

# 40. Why Bounded Queues Are Important

Never allow:

```text
unlimited frames
```

to accumulate.

If processing becomes slower than acquisition:

```text
Capture = 30 FPS
Processing = 10 FPS
```

memory can continuously grow.

Use a bounded ring buffer.

Example:

```text
Capacity = 5 frames
```

When full:

```text
drop oldest frame
```

or use a policy appropriate for the stream.

For live inspection, dropping old frames is usually preferable to processing increasingly stale frames.

---

# 41. Memory Overflow / Resource Protection

Monitor:

- RAM
- queue sizes
- allocation failures
- frame counts
- processing latency

Define limits.

Example:

```text
RGB queue > 80%:
warning

RGB queue = 100%:
drop oldest frame

RAM > configured threshold:
reduce buffering / enter degraded mode

Allocation failure:
log critical error
release temporary buffers
attempt recovery
```

Avoid repeatedly allocating large `cv::Mat` objects inside high-frequency loops where possible.

Prefer:

- buffer reuse
- preallocation
- bounded queues
- RAII
- smart pointers where ownership is dynamic

---

# 42. Camera Disconnect Handling

The application must detect:

```text
camera.open() failed
capture() failed
frame empty
repeated timeout
USB/V4L2 device disappears
```

State machine:

```text
CONNECTED
    |
    | capture failure
    v
DEGRADED
    |
    | retry
    v
RECONNECTING
    |
    +---- success ----> CONNECTED
    |
    +---- repeated failure
              |
              v
          FAULT
```

Do not crash the whole application for a recoverable camera failure.

---

# 43. Thermal Sensor / I2C Fault Handling

Detect:

- I2C read failure
- timeout
- invalid frame
- CRC/data integrity failure if applicable
- impossible temperature
- sensor initialization failure

Recovery:

```text
Read failure
    |
    v
Retry
    |
    +-- success --> continue
    |
    v
Reinitialize sensor
    |
    +-- success --> continue
    |
    v
Mark thermal unavailable
    |
    v
Degraded mode / alarm
```

---

# 44. Application Crash Handling

The application should be supervised by systemd.

If:

```text
thermal-inspection
       |
       X
    CRASH
```

systemd should restart it.

Example service policy:

```ini
[Service]
ExecStart=/usr/bin/thermal-inspection
Restart=on-failure
RestartSec=2
```

Use appropriate restart limits so a continuously crashing application does not restart forever without detection.

---

# 45. Application Health Monitoring

Expose internal health information:

```text
RGB camera:      OK
Thermal sensor:  OK
Synchronization: OK
Processing:      OK
Memory:          OK
Calibration:     OK
Last frame:      12 ms ago
FPS:             8.0
```

Possible internal state:

```cpp
enum class HealthState {
    OK,
    DEGRADED,
    FAULT
};
```

---

# 46. Watchdog Strategy

There should be a distinction between:

### Application-level recovery

The application detects and recovers from:

- camera disconnect
- sensor timeout
- queue overflow

### Process supervision

systemd restarts the application if it crashes or becomes unhealthy.

### Hardware watchdog

Optionally use the Raspberry Pi/Linux watchdog for severe system-level hangs.

The architecture should avoid depending on a single recovery mechanism.

---

# 47. Logging

Use structured logs.

Example:

```text
[INFO] Application started
[INFO] RGB camera initialized
[INFO] Thermal sensor initialized
[INFO] Calibration loaded
[INFO] Processing pipeline started

[WARN] Thermal frame timeout
[WARN] Reinitializing thermal sensor

[ERROR] RGB camera disconnected
[INFO] Attempting camera reconnect

[CRITICAL] Processing thread stopped
```

Store logs using Linux/systemd journaling in deployment.

---

# 48. System Initialization

At boot:

```text
Boot
 |
 v
Linux kernel
 |
 v
systemd
 |
 v
Hardware availability
 |
 v
Configuration
 |
 v
Calibration
 |
 v
RGB initialization
 |
 v
Thermal initialization
 |
 v
Processing threads
 |
 v
Health monitor
 |
 v
READY
```

The application should explicitly verify that:

- configuration exists
- calibration exists
- camera is accessible
- thermal sensor is accessible
- required directories exist
- sufficient resources exist

---

# 49. systemd Service

Create:

```text
thermal-inspection.service
```

Conceptually:

```ini
[Unit]
Description=Thermal PCB Inspection System
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/thermal-inspection
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Later add appropriate device/network dependencies.

Enable:

```bash
systemctl enable thermal-inspection.service
```

Start:

```bash
systemctl start thermal-inspection.service
```

Inspect:

```bash
systemctl status thermal-inspection.service
journalctl -u thermal-inspection.service
```

---

# 50. Yocto Integration

Create your own layer:

```text
meta-pcb-inspection
```

The layer should contain:

```text
recipes-app/
recipes-core/
recipes-systemd/
recipes-kernel/
```

The application should be packaged with a BitBake recipe.

The recipe should:

- fetch source
- configure CMake
- compile
- install binary
- install configuration
- install systemd service
- enable service if desired

---

# 51. Yocto Image

Create a custom image:

```text
pcb-inspection-image
```

It should include:

```text
Linux kernel
BusyBox/core utilities
systemd
C++ runtime
OpenCV
application
configuration
logging
networking
```

Eventually:

```text
bitbake pcb-inspection-image
```

produces a bootable Embedded Linux image.

---

# 52. Development Without Raspberry Pi

Use QEMU for the Embedded Linux portion.

```text
PC
 |
 v
Yocto build
 |
 v
Custom image
 |
 v
QEMU
 |
 v
Embedded Linux
 |
 v
Thermal inspection application
```

QEMU cannot reproduce every Raspberry Pi peripheral, so hardware-specific I2C/camera behavior must remain abstracted and tested with mocks/simulation.

---

# 53. Cross Compilation

Eventually build:

```text
x86_64 development machine
        |
        v
Yocto toolchain
        |
        v
ARM target
        |
        v
Raspberry Pi
```

The application should therefore avoid accidental dependencies on x86-specific behavior.

---

# 54. Hardware Driver Layer

Final hardware mode should have:

```text
Application
    |
    v
IThermalCamera
    |
    v
MLX90640Camera
    |
    v
I2C abstraction
    |
    v
Linux I2C
    |
    v
/dev/i2c-X
    |
    v
MLX90640
```

The same structure should exist for RGB acquisition.

---

# 55. Driver Responsibilities

The thermal driver should handle:

- Sensor initialization
- I2C configuration
- Register access
- Frame acquisition
- Sensor calibration data
- Temperature conversion
- Timeout detection
- Invalid data detection
- Reinitialization

The application should NOT directly manipulate I2C registers.

---

# 56. Configuration

Use an external configuration file rather than hardcoding parameters.

Example:

```yaml
rgb:
  width: 1280
  height: 720
  fps: 30

thermal:
  width: 32
  height: 24
  fps: 8

processing:
  spatial_sigma: 3.0
  range_sigma: 15.0
  hotspot_threshold: 60.0

synchronization:
  max_timestamp_difference_ms: 50

system:
  queue_size: 5
  reconnect_delay_ms: 2000
```

This makes tuning possible without recompiling.

---

# 57. Automated Tests

Use GoogleTest.

## Thermal tests

```text
Normal temperature -> no fault
80 C hotspot -> fault
Noisy frame -> stable result
Dead pixel -> handled
Invalid frame -> rejected
```

## Registration tests

```text
Known point + known H
        |
        v
Expected transformed point
```

Compare calculated and expected coordinates.

## Synchronization tests

Test:

```text
matching timestamps
different frame rates
missing frames
large timestamp gap
```

## Fault tests

Test:

```text
camera disconnect
I2C timeout
reconnection
queue overflow
application exception
invalid configuration
missing calibration
```

---

# 58. Fault Injection Framework

Create a simulation component capable of intentionally producing failures.

Example:

```cpp
struct FaultConfig {
    bool disconnectRGB;
    bool disconnectThermal;
    bool timeoutI2C;
    bool corruptThermalFrame;
    bool overflowQueue;
    bool delayThermal;
};
```

This allows repeatable testing.

Example:

```text
Start simulation
      |
      v
Normal operation
      |
      v
Disconnect thermal camera
      |
      v
Verify detection
      |
      v
Verify reconnect
      |
      v
Verify normal operation
```

This is extremely valuable for demonstrating embedded reliability engineering.

---

# 59. Performance Benchmarking

Measure:

```text
RGB FPS
Thermal FPS
End-to-end latency
Registration time
JBU processing time
Hotspot detection time
CPU utilization
RAM usage
Queue depth
Startup time
Recovery time
```

Example benchmark report:

```text
Resolution: 1280x720
RGB FPS: 30
Thermal FPS: 8
Average processing latency: XX ms
JBU latency: XX ms
CPU: XX %
RAM: XX MB
```

Do not invent values. Measure them.

---

# 60. Optimization

After correctness:

1. Profile.
2. Find the bottleneck.
3. Optimize.
4. Benchmark again.

Potential optimizations:

- Avoid unnecessary image copies.
- Reuse buffers.
- Limit JBU neighborhood.
- Use `cv::Mat` efficiently.
- Parallelize independent processing.
- Reduce unnecessary conversions.
- Use ARM-specific optimizations later if justified.

Do not optimize before measuring.

---

# 61. Suggested JBU Optimization

Naively evaluating every thermal pixel for every RGB pixel can be expensive.

Use a local neighborhood.

For each high-resolution pixel:

```text
only evaluate thermal samples
inside a radius
```

Instead of:

```text
every thermal pixel in the entire image
```

This reduces complexity dramatically.

Possible future optimizations:

- Precompute spatial weights.
- Use lookup tables for Gaussian weights.
- Vectorize calculations.
- Process image tiles.
- Parallelize independent pixels/regions.

---

# 62. Data Recording

Create a recording format containing:

```text
RGB image
thermal temperature matrix
timestamp
calibration version
configuration version
```

Do not only save the colorized thermal image.

The raw temperature matrix is required for scientific reproducibility.

Example:

```text
recording/
|
+-- rgb/
|   +-- 000001.png
|   +-- 000002.png
|
+-- thermal/
|   +-- 000001.bin
|   +-- 000002.bin
|
+-- timestamps.csv
+-- metadata.yaml
```

---

# 63. Replay Engine

Replay recorded frames at:

- real-time speed
- 0.5x
- 2x
- frame-by-frame

This is useful for debugging.

Example:

```text
Replay frame 153
      |
      v
Fault appears
      |
      v
Pause
      |
      v
Inspect RGB/thermal registration
```

---

# 64. Networking / Telemetry

After the local system works, add telemetry.

Possible MQTT topics:

```text
pcb-inspection/status
pcb-inspection/temperature
pcb-inspection/fault
pcb-inspection/health
```

Example:

```json
{
  "timestamp": 123456789,
  "status": "FAULT",
  "max_temperature": 82.3,
  "component_x": 850,
  "component_y": 430
}
```

The communication layer must be independent from the core processing pipeline.

---

# 65. Security / Robustness Basics

For deployment:

- Do not run unnecessary services.
- Avoid running the application as root unless required.
- Restrict file permissions.
- Validate configuration.
- Validate incoming network data.
- Limit log growth.
- Use bounded buffers.
- Fail safely on malformed input.

---

# 66. Final End-to-End Pipeline

```text
                  SYSTEM BOOT
                       |
                       v
                  systemd
                       |
                       v
               Application Init
                       |
          +------------+------------+
          |                         |
          v                         v
      RGB Camera               Thermal Sensor
          |                         |
          v                         v
     RGB Capture              Thermal Capture
          |                         |
          v                         v
      RGB Buffer              Thermal Buffer
          |                         |
          +------------+------------+
                       |
                       v
                Frame Synchronizer
                       |
                       v
                Calibration Check
                       |
                       v
               Frame Registration
                       |
                       v
              Thermal Temperature Field
                       |
                       v
             RGB-Guided JBU
                       |
                       v
               High-Resolution T
                       |
             +---------+---------+
             |                   |
             v                   v
       Hotspot Detection    Visualization
             |                   |
             v                   v
       Fault Localization   Color Mapping
             |                   |
             +---------+---------+
                       |
                       v
                 Final Result
                       |
              +--------+--------+
              |                 |
              v                 v
           Display           MQTT/API
```

---

# 67. Failure Scenarios to Demonstrate

The final demo should intentionally demonstrate:

## Scenario 1 — Normal

```text
Both cameras connected
        |
        v
Normal inspection
```

## Scenario 2 — Thermal disconnect

```text
Thermal sensor disconnected
        |
        v
Error detected
        |
        v
Retry
        |
        v
Reinitialize
        |
        v
Recovered
```

## Scenario 3 — RGB disconnect

```text
RGB camera disconnected
        |
        v
Detection fails
        |
        v
Camera reconnect
        |
        v
Pipeline resumes
```

## Scenario 4 — Processing overload

```text
Processing slower than capture
        |
        v
Queue approaches limit
        |
        v
Old frames dropped
        |
        v
Memory remains bounded
```

## Scenario 5 — Application crash

```text
Application crash
        |
        v
systemd detects failure
        |
        v
Process restarted
        |
        v
Initialization
        |
        v
Pipeline resumes
```

## Scenario 6 — Invalid calibration

```text
Calibration missing/invalid
        |
        v
System refuses unsafe fusion
        |
        v
Diagnostic state
```

---

# 68. Development Order

Do not build everything simultaneously.

## Stage 1

C++ + CMake + OpenCV.

## Stage 2

Webcam abstraction.

## Stage 3

Fake thermal camera.

## Stage 4

Thermal visualization.

## Stage 5

Hotspot detection.

## Stage 6

PCB contour detection.

## Stage 7

Checkerboard calibration.

## Stage 8

RGB/thermal registration.

## Stage 9

Basic interpolation baseline.

## Stage 10

Joint Bilateral Upsampling.

## Stage 11

Thermal gradient enhancement.

## Stage 12

Component-aware visualization.

## Stage 13

Frame synchronization.

## Stage 14

Multithreaded pipeline.

## Stage 15

Fault injection.

## Stage 16

Automated tests.

## Stage 17

Benchmarking and optimization.

## Stage 18

Yocto application recipe.

## Stage 19

Custom Yocto image.

## Stage 20

systemd boot/startup.

## Stage 21

QEMU deployment.

## Stage 22

Real Raspberry Pi deployment.

## Stage 23

Real thermal sensor driver.

## Stage 24

Real calibration.

## Stage 25

Final hardware validation.

---

# 69. Final Portfolio Deliverables

The GitHub repository should contain:

### Software

- C++ application
- CMake build
- Hardware abstraction layer
- Fake thermal camera
- Mock I2C
- Replay engine
- Real camera implementation
- Real thermal driver
- Sensor synchronization
- Calibration
- JBU
- Fault detection
- Multithreading
- Fault recovery

### Embedded Linux

- Custom Yocto layer
- BitBake recipes
- Custom image
- systemd service
- Configuration
- QEMU deployment instructions

### Testing

- Unit tests
- Fault injection
- Regression datasets
- Benchmark scripts

### Documentation

- Architecture diagram
- Calibration methodology
- JBU mathematics
- Sensor synchronization
- Fault-tolerance design
- Hardware setup
- Deployment instructions
- Performance measurements
- Known limitations

---

# 70. Final CV-Level Description

After completing the project, a strong description would be:

> **Embedded Edge-Based Multi-Spectral PCB Fault Detection System**  
> Designed a C++/Embedded Linux inspection system combining RGB and low-resolution thermal sensing for component-level PCB anomaly detection. Implemented hardware abstraction and simulated sensors for hardware-independent development, timestamp-based sensor synchronization, automatic checkerboard calibration and RGB/thermal homography registration, RGB-guided Joint Bilateral Upsampling for edge-aware thermal reconstruction, thermal anomaly detection, and component-aware visualization. Built a custom Yocto Linux image with BitBake recipes and systemd startup/recovery, including fault handling for sensor disconnection, I2C failures, bounded-memory buffering, application crashes, and automatic service recovery. Developed automated tests, fault injection, replay capabilities, and performance benchmarks.

---

# 71. Core Engineering Concepts Demonstrated

The project should ultimately demonstrate these layers:

```text
APPLICATION
    |
COMPUTER VISION
    |
SENSOR FUSION
    |
SIGNAL / IMAGE PROCESSING
    |
MULTITHREADING
    |
HARDWARE ABSTRACTION
    |
DEVICE DRIVERS
    |
LINUX
    |
YOCTO / EMBEDDED LINUX
    |
SYSTEM RELIABILITY
    |
PHYSICAL HARDWARE
```

The central design principle throughout the project is:

> **Develop and validate the application independently of hardware, then replace simulation interfaces with real Linux drivers without changing the core processing pipeline.**
