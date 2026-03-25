# ZED IceoryX2 Camera Publisher

This sample demonstrates how to stream ZED camera frames to shared memory using the **iceoryx2** framework.

## Overview

This service:
- Detects all connected ZED cameras (including multiple ZED Mini units)
- Creates separate publishers for left and right images for each camera
- Publishes frames to namespaced IPC topics via iceoryx2 shared memory
- Supports multiple concurrent cameras with independent frame capture threads

## Topic Naming

Topics are namespaced by camera index for multi-camera setups:
- `/cams/zed1/left` - Left image from first ZED camera
- `/cams/zed1/right` - Right image from first ZED camera
- `/cams/zed2/left` - Left image from second ZED camera
- `/cams/zed2/right` - Right image from second ZED camera
- And so on...

## Image Data Format

Each published message contains:
```cpp
struct ImageData {
    uint32_t width;              // Image width
    uint32_t height;             // Image height
    uint32_t timestamp;          // Frame timestamp
    uint32_t frame_id;           // Frame counter
    uint32_t serial_number;      // Camera serial number
    uint8_t image_data[];        // RGBA image data (up to 4K resolution)
};
```

## Building

### Prerequisites
- ZED SDK 4.0+
- iceoryx2 library
- CUDA SDK
- CMake 3.5+

### Build Instructions

```bash
cd /path/to/zed-sdk
mkdir build && cd build
cmake ..
make
```

Or to build just this sample:
```bash
cd camera\ streaming/iceoryx2-publisher/cpp
mkdir build && cd build
cmake ..
make
```

The executable will be `ZED_IceoryX2_Camera_Publisher`

## Usage

```bash
./ZED_IceoryX2_Camera_Publisher
```

The service will:
1. Detect all connected ZED cameras
2. Print camera information (model, serial number)
3. Create iceoryx2 publishers for each camera's left and right images
4. Continuously capture and publish frames
5. Print statistics every 100 frames

To stop, press `Ctrl+C`.

## Subscriber Example

To consume frames from this publisher using iceoryx2, see the `zed-opencv` repository for a C++ subscriber example that demonstrates:
- Subscribing to the published topics
- Receiving frame data from shared memory
- Processing frames with OpenCV

## Notes

- Frame resolution follows the camera's auto-detection setting
- FPS is set to 30 by default (configurable in InitParameters)
- Only RGBA format is supported (4 bytes per pixel)
- History size is set to 1 (only latest frame retained)
- All timestamps are synchronized with the camera hardware clock

## Troubleshooting

### No cameras detected
- Check USB connections
- Verify cameras are recognized by `lsusb`
- Check camera permissions

### Publisher creation fails
- Ensure iceoryx2 middleware is properly installed
- Check iceoryx2 service configuration
- Verify sufficient shared memory is available

### Frame publishing issues
- Check that subscribers aren't holding locks
- Monitor CPU usage for frame drop indicators
- Verify image dimensions don't exceed 4K resolution
