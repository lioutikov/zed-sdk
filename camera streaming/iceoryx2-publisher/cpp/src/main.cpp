///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2025, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This sample demonstrates how to stream ZED camera frames to shared memory
 * using the iceoryx2 framework. Supports multiple cameras with namespaced topics:
 * /cams/zed1/left, /cams/zed1/right, /cams/zed2/left, etc.
 */

#include <sl/Camera.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <csignal>

// iceoryx2 C API includes
#include "iceoryx2/c/api.h"

using namespace sl;
using namespace std;

static atomic<bool> g_exit_app(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_exit_app = true;
    }
}

struct ImageData {
    static constexpr size_t HEADER_SIZE = 256;
    static constexpr size_t MAX_IMAGE_SIZE = 4096 * 2160 * 4; // RGBA format for 4K
    static constexpr size_t TOTAL_SIZE = HEADER_SIZE + MAX_IMAGE_SIZE;

    uint32_t width;
    uint32_t height;
    uint32_t timestamp;
    uint32_t frame_id;
    uint32_t serial_number;
    uint8_t padding[HEADER_SIZE - 16];
    uint8_t image_data[MAX_IMAGE_SIZE];
};

struct CameraPublisher {
    int camera_id;
    uint32_t serial_number;
    Camera camera;

    // iceoryx2 publishers for left and right images
    iox2_publisher_h publisher_left;
    iox2_publisher_h publisher_right;

    bool is_open;
    uint32_t frame_count;
};

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // Initialize ZED SDK
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO;
    init_parameters.depth_mode = DEPTH_MODE::NONE;
    init_parameters.camera_fps = 30;

    // Detect all connected cameras
    vector<DeviceProperties> device_list = Camera::getDeviceList();
    int num_cameras = device_list.size();

    cout << "[ZED-Iceoryx2] Found " << num_cameras << " ZED camera(s)" << endl;

    if (num_cameras == 0) {
        cerr << "[ZED-Iceoryx2] No ZED cameras detected. Exiting." << endl;
        return EXIT_FAILURE;
    }

    // Print detected cameras
    for (int i = 0; i < num_cameras; i++) {
        cout << "[ZED-Iceoryx2] Camera " << i << ": Model=" << device_list[i].camera_model
             << ", Serial=" << device_list[i].serial_number
             << ", State=" << device_list[i].camera_state << endl;
    }

    // Initialize iceoryx2 runtime
    iox2_runtime_h runtime = iox2_runtime_init(NULL);
    if (runtime == NULL) {
        cerr << "[ZED-Iceoryx2] Failed to initialize iceoryx2 runtime" << endl;
        return EXIT_FAILURE;
    }
    cout << "[ZED-Iceoryx2] Iceoryx2 runtime initialized" << endl;

    // Create camera publishers
    vector<CameraPublisher> publishers;

    for (int cam_idx = 0; cam_idx < num_cameras; cam_idx++) {
        CameraPublisher pub;
        pub.camera_id = cam_idx;
        pub.serial_number = device_list[cam_idx].serial_number;
        pub.is_open = false;
        pub.frame_count = 0;

        // Initialize the camera
        init_parameters.input.setFromSerialNumber(device_list[cam_idx].serial_number);
        ERROR_CODE err = pub.camera.open(init_parameters);

        if (err <= ERROR_CODE::SUCCESS) {
            auto cam_info = pub.camera.getCameraInformation();
            cout << "[ZED-Iceoryx2] Opened Camera ID=" << cam_idx
                 << ", Serial=" << cam_info.serial_number << endl;
            pub.is_open = true;

            // Create iceoryx2 service names with namespace
            string service_name_left = "/cams/zed" + to_string(cam_idx + 1) + "/left";
            string service_name_right = "/cams/zed" + to_string(cam_idx + 1) + "/right";

            // Create publishers for left image
            iox2_pub_sub_options_t options = iox2_pub_sub_options_default();
            options.history_size = 1; // Keep only the latest frame
            options.max_subscribers = 10;

            pub.publisher_left = iox2_node_create_publisher(
                runtime,
                service_name_left.c_str(),
                &ImageData::TOTAL_SIZE,
                &options
            );

            if (pub.publisher_left == NULL) {
                cerr << "[ZED-Iceoryx2] Failed to create publisher for " << service_name_left << endl;
            } else {
                cout << "[ZED-Iceoryx2] Created publisher: " << service_name_left << endl;
            }

            // Create publishers for right image
            pub.publisher_right = iox2_node_create_publisher(
                runtime,
                service_name_right.c_str(),
                &ImageData::TOTAL_SIZE,
                &options
            );

            if (pub.publisher_right == NULL) {
                cerr << "[ZED-Iceoryx2] Failed to create publisher for " << service_name_right << endl;
            } else {
                cout << "[ZED-Iceoryx2] Created publisher: " << service_name_right << endl;
            }

            publishers.push_back(pub);
        } else {
            cerr << "[ZED-Iceoryx2] Failed to open camera " << cam_idx << ", Error: " << err << endl;
        }
    }

    if (publishers.empty()) {
        cerr << "[ZED-Iceoryx2] No cameras opened successfully. Exiting." << endl;
        return EXIT_FAILURE;
    }

    cout << "[ZED-Iceoryx2] Starting frame capture and publishing..." << endl;
    cout << "[ZED-Iceoryx2] Press Ctrl+C to stop" << endl;

    // Main frame capture and publishing loop
    Mat left_image, right_image;
    int iteration = 0;

    while (!g_exit_app) {
        for (auto& pub : publishers) {
            if (!pub.is_open) continue;

            // Grab frames from the camera
            if (pub.camera.grab() <= ERROR_CODE::SUCCESS) {
                pub.frame_count++;

                // Retrieve left image
                if (pub.camera.retrieveImage(left_image, VIEW::LEFT, MEM::CPU) == ERROR_CODE::SUCCESS) {
                    if (pub.publisher_left != NULL) {
                        // Allocate and fill data for left image
                        void* payload = iox2_publisher_loan_uninit(pub.publisher_left);
                        if (payload != NULL) {
                            ImageData* data = static_cast<ImageData*>(payload);
                            data->width = left_image.getWidth();
                            data->height = left_image.getHeight();
                            data->timestamp = pub.camera.getTimestamp(TIME_REFERENCE::IMAGE);
                            data->frame_id = pub.frame_count;
                            data->serial_number = pub.serial_number;

                            // Copy image data
                            size_t image_bytes = left_image.getWidth() * left_image.getHeight() * 4;
                            if (image_bytes <= ImageData::MAX_IMAGE_SIZE) {
                                memcpy(data->image_data, left_image.getPtr<uint8_t>(MEM::CPU), image_bytes);
                                iox2_publisher_send(pub.publisher_left, payload);
                            } else {
                                cerr << "[ZED-Iceoryx2] Image too large for buffer" << endl;
                                iox2_publisher_release_loan(pub.publisher_left, payload);
                            }
                        }
                    }
                }

                // Retrieve right image
                if (pub.camera.retrieveImage(right_image, VIEW::RIGHT, MEM::CPU) == ERROR_CODE::SUCCESS) {
                    if (pub.publisher_right != NULL) {
                        // Allocate and fill data for right image
                        void* payload = iox2_publisher_loan_uninit(pub.publisher_right);
                        if (payload != NULL) {
                            ImageData* data = static_cast<ImageData*>(payload);
                            data->width = right_image.getWidth();
                            data->height = right_image.getHeight();
                            data->timestamp = pub.camera.getTimestamp(TIME_REFERENCE::IMAGE);
                            data->frame_id = pub.frame_count;
                            data->serial_number = pub.serial_number;

                            // Copy image data
                            size_t image_bytes = right_image.getWidth() * right_image.getHeight() * 4;
                            if (image_bytes <= ImageData::MAX_IMAGE_SIZE) {
                                memcpy(data->image_data, right_image.getPtr<uint8_t>(MEM::CPU), image_bytes);
                                iox2_publisher_send(pub.publisher_right, payload);
                            } else {
                                cerr << "[ZED-Iceoryx2] Image too large for buffer" << endl;
                                iox2_publisher_release_loan(pub.publisher_right, payload);
                            }
                        }
                    }
                }
            }
        }

        // Log statistics every 100 iterations
        iteration++;
        if (iteration % 100 == 0) {
            cout << "[ZED-Iceoryx2] " << iteration << " iterations - ";
            for (size_t i = 0; i < publishers.size(); i++) {
                cout << "Camera" << i << "=" << publishers[i].frame_count << " ";
            }
            cout << endl;
        }

        // Small sleep to avoid busy waiting
        this_thread::sleep_for(chrono::milliseconds(1));
    }

    cout << "[ZED-Iceoryx2] Shutting down..." << endl;

    // Cleanup
    for (auto& pub : publishers) {
        if (pub.publisher_left != NULL) {
            iox2_publisher_drop(pub.publisher_left);
        }
        if (pub.publisher_right != NULL) {
            iox2_publisher_drop(pub.publisher_right);
        }
        if (pub.is_open) {
            pub.camera.close();
        }
    }

    // Cleanup iceoryx2 runtime
    iox2_runtime_drop(runtime);

    cout << "[ZED-Iceoryx2] Shutdown complete" << endl;
    return EXIT_SUCCESS;
}
