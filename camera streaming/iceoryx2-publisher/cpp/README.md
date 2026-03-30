# ZED IceoryX2 Camera Publisher

This sample streams ZED camera images over **iceoryx2** shared memory using the current C++ API. It is designed for current `iceoryx2` releases and current ZED SDK usage patterns.

## Overview

The publisher:
- Detects all connected ZED cameras
- Can list available ZED cameras with detailed device metadata
- Can restrict streaming to a selected subset of cameras by serial or index
- Avoids opening or probing filtered-out cameras during streaming
- Opens one left and one right image publisher per camera
- Uses the `iceoryx2` C++ API with RAII objects instead of raw C handles
- Publishes image bytes as a dynamic `Slice<uint8_t>` payload
- Publishes frame metadata in a typed user header
- Exposes runtime camera controls through an `iceoryx2` request/response service
- Exposes stereo calibration snapshots through an `iceoryx2` blackboard service
- Uses `Node::wait(...)` for pacing and shutdown instead of a manual busy loop

## Service Naming

Services are namespaced by camera index:
- `cams/zed1/left`
- `cams/zed1/right`
- `cams/zed1/control`
- `cams/zed1/calibration`
- `cams/zed2/left`
- `cams/zed2/right`
- `cams/zed2/control`
- `cams/zed2/calibration`

You can change the `cams` prefix with `--service-prefix`.

Aggregate IPC services:
- Fleet control service: `cams/control`
- Fleet calibration blackboard: `cams/calibration`

Why these protocols:
- `request/response` is used for camera settings because callers need explicit success/error feedback and read-after-write responses.
- `blackboard` is used for calibration because it represents the latest state per stereo pair and should be readable at any time without replaying a stream.
- Per-camera control and calibration paths keep each camera instance self-contained.
- Aggregate services provide fleet-wide access and can fan out across instance services.

## Wire Format

Each sample contains:
- Payload: raw image bytes from a ZED `sl::Mat`
- User header: fixed-size metadata

```cpp
struct FrameHeader {
    uint64_t timestamp_ns;
    uint64_t frame_id;
    uint32_t serial_number;
    uint32_t camera_index;
    uint32_t camera_model;
    uint32_t view;
    uint32_t width;
    uint32_t height;
    uint32_t step_bytes;
    uint32_t image_bytes;
    uint32_t channels;
    uint32_t bytes_per_pixel;
    uint32_t mat_type;
    uint32_t reserved;
};
```

This avoids a large fixed 4K buffer in every shared-memory sample and scales with the actual frame size being sent.

## Build

### Prerequisites
- ZED SDK 5.2.2 or compatible
- iceoryx2 with C++ bindings installed
- CUDA SDK
- CMake 3.22+

### Build Instructions

```bash
cd camera\ streaming/iceoryx2-publisher/cpp
mkdir -p build
cd build
cmake ..
make -j
```

The executable will be `ZED_IceoryX2_Camera_Publisher`.

## Usage

### Basic

```bash
./ZED_IceoryX2_Camera_Publisher
```

### Common Options

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-resolution HD720 \
  --camera-fps 30 \
  --camera-serials 12345678,87654321 \
  --service-prefix cams \
  --max-subscribers 16
```

### Publish Unrectified Views

```bash
./ZED_IceoryX2_Camera_Publisher --unrectified
```

### Help

```bash
./ZED_IceoryX2_Camera_Publisher --help
```

### List Available Cameras

```bash
./ZED_IceoryX2_Camera_Publisher --list-cameras
```

This prints the detected device index, serial number, model, path, video device, badge/sensor metadata, and best-effort opened camera information such as firmware, active resolution, and FPS.

### Stream Only Selected Cameras

```bash
./ZED_IceoryX2_Camera_Publisher --camera-indices 0,2
```

or

```bash
./ZED_IceoryX2_Camera_Publisher --camera-serials 12345678,87654321
```

When either selector is provided, the streamer still performs device discovery so it can map serials and indices, but it only opens and starts the matched cameras.

You can also target a single stream camera with:

```bash
./ZED_IceoryX2_Camera_Publisher --camera-target serial:12345678
```

### Query Runtime Camera Settings Over IPC

With the streamer already running:

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-target serial:12345678 \
  --list-settings
```

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-target serial:12345678 \
  --get-setting exposure
```

### Change Runtime Camera Settings Over IPC

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-target serial:12345678 \
  --set-setting exposure=42
```

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-target serial:12345678 \
  --set-setting-range auto_exposure_time_range=2000:5000
```

```bash
./ZED_IceoryX2_Camera_Publisher \
  --camera-target serial:12345678 \
  --reset-settings
```

Supported runtime settings intentionally exclude image-geometry changes such as acquisition resolution, publish scaling, and cropping.

### Read Calibration Over IPC

```bash
./ZED_IceoryX2_Camera_Publisher --list-calibrations
```

```bash
./ZED_IceoryX2_Camera_Publisher --read-calibration 12345678
```

Calibration snapshots are keyed by camera serial on the `cams/calibration` blackboard and include:
- Active published resolution
- Whether the stream is rectified or unrectified
- Rectified and raw left/right intrinsics
- Stereo transform and baseline
- Firmware metadata

When a single camera is targeted, the CLI resolves and uses that camera's direct service namespace:
- Control: `cams/zedN/control`
- Calibration: `cams/zedN/calibration`

The aggregate services remain available for fleet-wide operations such as:
- `--camera-target all --list-settings`
- `--camera-target all --reset-settings`
- `--list-calibrations`

## Notes

- The publisher uses `sl::RuntimeParameters` during `grab(...)`.
- Image transport is dynamic: the shared-memory loan matches the actual frame byte count.
- The published byte layout is whatever `sl::Mat` exposes for the selected ZED view and output resolution.
- For `VIEW::LEFT` and `VIEW::RIGHT`, ZED returns BGRA data by default.
- History defaults to `1` and safe overflow is enabled for the service.
- Publisher allocation uses the `iceoryx2` power-of-two growth strategy.
- Calibration snapshots are generated with `getCameraInformation(requested_resolution)`, so they already follow the effective published resolution.
- Cropping is not part of the runtime IPC controls; if added later, principal point / intrinsics adjustments should be applied in this publisher before updating the blackboard snapshot.
- Default acquisition mode is `HD720` at `30 FPS`.

## Subscriber Expectations

A subscriber should:
- Open the same `iceoryx2` service name
- Expect payload type `Slice<uint8_t>`
- Expect user header type `FrameHeader`
- Interpret payload bytes using `FrameHeader.width`, `height`, `step_bytes`, `channels`, and `mat_type`

## Troubleshooting

### No cameras detected
- Check USB connections
- Verify camera permissions
- Confirm the ZED SDK tools can see the cameras

### Publisher creation fails
- Ensure the `iceoryx2-cxx` package is installed and discoverable by CMake
- Check shared-memory limits and `iceoryx2` configuration
- Make sure service names are valid semantic strings

### Subscribers misread image data
- Confirm the subscriber matches the `FrameHeader` layout exactly
- Use `step_bytes` instead of assuming tightly packed rows
- Check whether the publisher is using rectified or unrectified views
