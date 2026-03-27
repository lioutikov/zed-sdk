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
 *
 * Additional functionality:
 * - camera discovery / inventory listing
 * - selecting a subset of cameras by serial or device index
 * - runtime camera control via iceoryx2 request/response
 * - stereo calibration snapshots via iceoryx2 blackboard
 */

#include <sl/Camera.hpp>

#include "iox2/iceoryx2.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

struct FrameHeader {
    static constexpr const char IOX2_TYPE_NAME[] = "zed_sdk::FrameHeaderV1";

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

struct CameraIntrinsics {
    static constexpr const char IOX2_TYPE_NAME[] = "zed_sdk::CameraIntrinsicsV1";

    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
    float h_fov_deg = 0.0F;
    float v_fov_deg = 0.0F;
    float d_fov_deg = 0.0F;
    float focal_length_metric = 0.0F;
    uint32_t width = 0U;
    uint32_t height = 0U;
    std::array<double, 12U> disto {};
};

struct StereoCalibrationSnapshot {
    static constexpr const char IOX2_TYPE_NAME[] = "zed_sdk::StereoCalibrationSnapshotV1";

    uint32_t serial_number = 0U;
    uint32_t camera_index = 0U;
    uint32_t camera_model = 0U;
    uint32_t firmware_version = 0U;
    uint32_t sensors_firmware_version = 0U;
    uint32_t active_width = 0U;
    uint32_t active_height = 0U;
    uint32_t crop_offset_x = 0U;
    uint32_t crop_offset_y = 0U;
    uint32_t crop_width = 0U;
    uint32_t crop_height = 0U;
    uint32_t is_rectified_stream = 0U;
    uint32_t has_publish_resolution_override = 0U;
    float fps = 0.0F;
    float rectified_baseline = 0.0F;
    float raw_baseline = 0.0F;
    std::array<float, 16U> rectified_stereo_transform {};
    std::array<float, 16U> raw_stereo_transform {};
    CameraIntrinsics rectified_left {};
    CameraIntrinsics rectified_right {};
    CameraIntrinsics raw_left {};
    CameraIntrinsics raw_right {};
};

enum class ProgramMode : uint32_t {
    Stream = 0U,
    ListCameras = 1U,
    ControlClient = 2U,
    CalibrationClient = 3U,
};

enum class CameraSelectorKind : uint32_t {
    All = 0U,
    Serial = 1U,
    Index = 2U,
};

enum class ControlCommand : uint32_t {
    None = 0U,
    ListSettings = 1U,
    GetSetting = 2U,
    SetSetting = 3U,
    SetRange = 4U,
    ResetSettings = 5U,
};

enum class ControlStatus : uint32_t {
    Ok = 0U,
    InvalidRequest = 1U,
    CameraNotFound = 2U,
    UnsupportedSetting = 3U,
    ZedError = 4U,
};

enum class SettingValueShape : uint32_t {
    Scalar = 0U,
    Range = 1U,
    ReadOnlyScalar = 2U,
};

struct ControlRequest {
    static constexpr const char IOX2_TYPE_NAME[] = "zed_sdk::CameraControlRequestV1";

    uint32_t command = static_cast<uint32_t>(ControlCommand::None);
    uint32_t selector_kind = static_cast<uint32_t>(CameraSelectorKind::All);
    uint32_t selector_value = 0U;
    uint32_t setting = std::numeric_limits<uint32_t>::max();
    int32_t value = 0;
    int32_t value_second = 0;
    uint32_t reserved = 0U;
};

struct ControlResponse {
    static constexpr const char IOX2_TYPE_NAME[] = "zed_sdk::CameraControlResponseV1";

    uint32_t status = static_cast<uint32_t>(ControlStatus::Ok);
    uint32_t command = static_cast<uint32_t>(ControlCommand::None);
    uint32_t serial_number = 0U;
    uint32_t camera_index = 0U;
    uint32_t camera_model = 0U;
    uint32_t setting = std::numeric_limits<uint32_t>::max();
    uint32_t shape = static_cast<uint32_t>(SettingValueShape::Scalar);
    int32_t value = 0;
    int32_t value_second = 0;
    int32_t min_value = 0;
    int32_t max_value = 0;
    int32_t zed_error_code = 0;
    char setting_name[48] {};
    char message[160] {};
};

using BytePayload = iox2::bb::Slice<uint8_t>;
using IpcNode = iox2::Node<iox2::ServiceType::Ipc>;
using IpcService = iox2::PortFactoryPublishSubscribe<iox2::ServiceType::Ipc, BytePayload, FrameHeader>;
using IpcPublisher = iox2::Publisher<iox2::ServiceType::Ipc, BytePayload, FrameHeader>;
using ControlService = iox2::PortFactoryRequestResponse<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>;
using ControlServer = iox2::Server<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>;
using ControlClient = iox2::Client<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>;
using CalibrationService = iox2::PortFactoryBlackboard<iox2::ServiceType::Ipc, uint32_t>;
using CalibrationWriter = iox2::Writer<iox2::ServiceType::Ipc, uint32_t>;
using CalibrationReader = iox2::Reader<iox2::ServiceType::Ipc, uint32_t>;

static_assert(std::is_trivially_copyable<FrameHeader>::value, "FrameHeader must be trivially copyable");
static_assert(std::is_trivially_copyable<ControlRequest>::value, "ControlRequest must be trivially copyable");
static_assert(std::is_trivially_copyable<ControlResponse>::value, "ControlResponse must be trivially copyable");
static_assert(std::is_trivially_copyable<StereoCalibrationSnapshot>::value,
              "StereoCalibrationSnapshot must be trivially copyable");

struct Options {
    ProgramMode mode = ProgramMode::Stream;
    int camera_fps = 30;
    sl::RESOLUTION camera_resolution = sl::RESOLUTION::AUTO;
    std::optional<sl::Resolution> publish_resolution;
    std::string service_prefix = "cams";
    std::string node_name = "zed_iceoryx2_camera_publisher";
    uint64_t history_size = 1U;
    uint64_t max_subscribers = 10U;
    uint64_t initial_slice_hint = 8U * 1024U * 1024U;
    bool rectified = true;
    std::vector<uint32_t> selected_serials;
    std::vector<size_t> selected_indices;

    CameraSelectorKind control_selector_kind = CameraSelectorKind::All;
    uint32_t control_selector_value = 0U;
    ControlCommand control_command = ControlCommand::None;
    std::optional<sl::VIDEO_SETTINGS> control_setting;
    int32_t control_value = 0;
    int32_t control_value_second = 0;
    bool calibration_list_all = false;
    std::optional<uint32_t> calibration_serial;
};

struct StreamEndpoint {
    std::string service_name;
    sl::VIEW view = sl::VIEW::LEFT;
    std::optional<IpcService> service;
    std::optional<IpcPublisher> publisher;
};

struct CameraContext {
    size_t device_index = 0U;
    uint32_t serial_number = 0U;
    sl::MODEL camera_model = sl::MODEL::ZED;
    sl::Camera camera;
    uint64_t frame_count = 0U;
    uint64_t published_frames = 0U;
    StreamEndpoint left;
    StreamEndpoint right;
    StereoCalibrationSnapshot calibration {};
};

struct ControlChannel {
    std::string service_name;
    std::optional<ControlService> service;
    std::optional<ControlServer> server;
};

struct CalibrationChannel {
    std::string service_name;
    std::optional<CalibrationService> service;
    std::optional<CalibrationWriter> writer;
};

struct SettingSpec {
    sl::VIDEO_SETTINGS setting;
    const char* name;
    SettingValueShape shape;
    bool writable;
};

constexpr uint32_t INVALID_SETTING_ID = std::numeric_limits<uint32_t>::max();

constexpr std::array<SettingSpec, 16U> kSettingSpecs {{
    {sl::VIDEO_SETTINGS::BRIGHTNESS, "brightness", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::CONTRAST, "contrast", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::HUE, "hue", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::SATURATION, "saturation", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::SHARPNESS, "sharpness", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::GAMMA, "gamma", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::GAIN, "gain", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::EXPOSURE, "exposure", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::AEC_AGC, "aec_agc", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, "whitebalance_temperature", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, "whitebalance_auto", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::LED_STATUS, "led_status", SettingValueShape::Scalar, true},
    {sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE, "auto_exposure_time_range", SettingValueShape::Range, true},
    {sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE, "auto_analog_gain_range", SettingValueShape::Range, true},
    {sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE, "auto_digital_gain_range", SettingValueShape::Range, true},
    {sl::VIDEO_SETTINGS::SCENE_ILLUMINANCE, "scene_illuminance", SettingValueShape::ReadOnlyScalar, false},
}};

auto print_usage(const char* program_name) -> void {
    std::cout
        << "Usage: " << program_name << " [options]\n"
        << "\n"
        << "Streaming options:\n"
        << "  --camera-fps <fps>                   Camera FPS (default: 30)\n"
        << "  --camera-resolution <name>           AUTO, HD720, HD1080, HD1200, 2K\n"
        << "  --publish-resolution <WxH>           Resize published images before transport\n"
        << "  --camera-serials <s1,s2,...>         Use only the listed serial numbers\n"
        << "  --camera-indices <i1,i2,...>         Use only the listed device indices from --list-cameras\n"
        << "  --service-prefix <prefix>            Service prefix without leading slash (default: cams)\n"
        << "  --node-name <name>                   iceoryx2 node name\n"
        << "  --history-size <n>                   iceoryx2 history size (default: 1)\n"
        << "  --max-subscribers <n>                Max subscribers per image service (default: 10)\n"
        << "  --initial-slice-hint <bytes>         Initial shared-memory slice hint (default: 8388608)\n"
        << "  --unrectified                        Publish LEFT_UNRECTIFIED / RIGHT_UNRECTIFIED\n"
        << "\n"
        << "Discovery modes:\n"
        << "  --list-cameras                       List detected ZED cameras and exit\n"
        << "  --list-calibrations                  Read all published calibration snapshots and exit\n"
        << "  --read-calibration <serial>          Read one published calibration snapshot and exit\n"
        << "\n"
        << "IPC control client modes:\n"
        << "  --camera-target <all|serial:N|index:N>\n"
        << "                                       Select control target for commands below\n"
        << "  --list-settings                      List supported runtime settings for the target camera(s)\n"
        << "  --get-setting <name>                 Query one runtime setting via IPC\n"
        << "  --set-setting <name=value>           Set one scalar runtime setting via IPC\n"
        << "  --set-setting-range <name=min:max>   Set one range runtime setting via IPC\n"
        << "  --reset-settings                     Reset supported runtime settings to defaults via IPC\n"
        << "\n"
        << "Notes:\n"
        << "  - Runtime IPC settings intentionally exclude frame geometry changes such as resolution,\n"
        << "    scaling, and cropping.\n"
        << "  - Image services are published as <prefix>/zedN/left and <prefix>/zedN/right.\n"
        << "  - Camera control service is <prefix>/control.\n"
        << "  - Calibration blackboard service is <prefix>/calibration.\n"
        << "  --help                               Show this help\n";
}

auto to_lower(std::string value) -> std::string {
    std::transform(value.begin(),
                   value.end(),
                   value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

auto trim_slashes(std::string value) -> std::string {
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    while (!value.empty() && value.back() == '/') {
        value.pop_back();
    }
    return value;
}

auto parse_u64(const std::string& value, const std::string& option_name) -> uint64_t {
    try {
        return std::stoull(value);
    } catch (...) {
        throw std::runtime_error("Invalid numeric value for " + option_name + ": " + value);
    }
}

auto parse_u32(const std::string& value, const std::string& option_name) -> uint32_t {
    const auto parsed = parse_u64(value, option_name);
    if (parsed > static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        throw std::runtime_error("Value out of range for " + option_name + ": " + value);
    }
    return static_cast<uint32_t>(parsed);
}

auto parse_int(const std::string& value, const std::string& option_name) -> int {
    try {
        return std::stoi(value);
    } catch (...) {
        throw std::runtime_error("Invalid integer value for " + option_name + ": " + value);
    }
}

template <typename ValueType, typename Parser>
auto parse_csv_list(const std::string& value, const std::string& option_name, Parser&& parser) -> std::vector<ValueType> {
    std::vector<ValueType> result;
    size_t start = 0U;

    while (start <= value.size()) {
        const auto comma_pos = value.find(',', start);
        const auto token = value.substr(start, comma_pos == std::string::npos ? std::string::npos : comma_pos - start);
        if (token.empty()) {
            throw std::runtime_error("Empty element in " + option_name);
        }
        result.push_back(parser(token, option_name));

        if (comma_pos == std::string::npos) {
            break;
        }
        start = comma_pos + 1U;
    }

    return result;
}

auto parse_camera_resolution(const std::string& value) -> sl::RESOLUTION {
    const auto normalized = to_lower(value);
    if (normalized == "auto") {
        return sl::RESOLUTION::AUTO;
    }
    if (normalized == "hd720") {
        return sl::RESOLUTION::HD720;
    }
    if (normalized == "hd1080") {
        return sl::RESOLUTION::HD1080;
    }
    if (normalized == "hd1200") {
        return sl::RESOLUTION::HD1200;
    }
    if (normalized == "2k" || normalized == "hd2k") {
        return sl::RESOLUTION::HD2K;
    }

    throw std::runtime_error("Unsupported camera resolution: " + value);
}

auto parse_publish_resolution(const std::string& value) -> sl::Resolution {
    const auto separator_pos = value.find('x');
    if (separator_pos == std::string::npos) {
        throw std::runtime_error("Publish resolution must use WIDTHxHEIGHT format");
    }

    const auto width = parse_int(value.substr(0, separator_pos), "--publish-resolution");
    const auto height = parse_int(value.substr(separator_pos + 1), "--publish-resolution");
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Publish resolution dimensions must be positive");
    }

    return sl::Resolution(static_cast<size_t>(width), static_cast<size_t>(height));
}

auto find_setting_spec(sl::VIDEO_SETTINGS setting) -> const SettingSpec* {
    for (const auto& spec : kSettingSpecs) {
        if (spec.setting == setting) {
            return &spec;
        }
    }
    return nullptr;
}

auto find_setting_spec_by_name(const std::string& value) -> const SettingSpec* {
    const auto normalized = to_lower(value);
    for (const auto& spec : kSettingSpecs) {
        if (normalized == spec.name || normalized == to_lower(std::string(sl::toString(spec.setting).c_str()))) {
            return &spec;
        }
    }
    return nullptr;
}

auto parse_camera_target(const std::string& value, CameraSelectorKind& kind, uint32_t& selector_value) -> void {
    const auto normalized = to_lower(value);
    if (normalized == "all") {
        kind = CameraSelectorKind::All;
        selector_value = 0U;
        return;
    }

    constexpr const char serial_prefix[] = "serial:";
    constexpr const char index_prefix[] = "index:";
    if (normalized.rfind(serial_prefix, 0U) == 0U) {
        kind = CameraSelectorKind::Serial;
        selector_value = parse_u32(value.substr(std::strlen(serial_prefix)), "--camera-target");
        return;
    }
    if (normalized.rfind(index_prefix, 0U) == 0U) {
        kind = CameraSelectorKind::Index;
        selector_value = parse_u32(value.substr(std::strlen(index_prefix)), "--camera-target");
        return;
    }

    throw std::runtime_error("Invalid --camera-target. Expected all, serial:<n>, or index:<n>");
}

auto parse_options(int argc, char** argv) -> Options {
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        auto require_value = [&](const std::string& option_name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + option_name);
            }
            return argv[++i];
        };

        if (arg == "--help") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else if (arg == "--camera-fps") {
            options.camera_fps = parse_int(require_value(arg), arg);
        } else if (arg == "--camera-resolution") {
            options.camera_resolution = parse_camera_resolution(require_value(arg));
        } else if (arg == "--publish-resolution") {
            options.publish_resolution = parse_publish_resolution(require_value(arg));
        } else if (arg == "--camera-serials") {
            options.selected_serials =
                parse_csv_list<uint32_t>(require_value(arg), arg, [](const std::string& token, const std::string& name) {
                    return parse_u32(token, name);
                });
        } else if (arg == "--camera-indices") {
            options.selected_indices =
                parse_csv_list<size_t>(require_value(arg), arg, [](const std::string& token, const std::string& name) {
                    return static_cast<size_t>(parse_u64(token, name));
                });
        } else if (arg == "--service-prefix") {
            options.service_prefix = trim_slashes(require_value(arg));
        } else if (arg == "--node-name") {
            options.node_name = require_value(arg);
        } else if (arg == "--history-size") {
            options.history_size = parse_u64(require_value(arg), arg);
        } else if (arg == "--max-subscribers") {
            options.max_subscribers = parse_u64(require_value(arg), arg);
        } else if (arg == "--initial-slice-hint") {
            options.initial_slice_hint = parse_u64(require_value(arg), arg);
        } else if (arg == "--unrectified") {
            options.rectified = false;
        } else if (arg == "--list-cameras") {
            options.mode = ProgramMode::ListCameras;
        } else if (arg == "--camera-target") {
            parse_camera_target(require_value(arg), options.control_selector_kind, options.control_selector_value);
        } else if (arg == "--list-settings") {
            options.mode = ProgramMode::ControlClient;
            options.control_command = ControlCommand::ListSettings;
        } else if (arg == "--get-setting") {
            options.mode = ProgramMode::ControlClient;
            options.control_command = ControlCommand::GetSetting;
            const auto* spec = find_setting_spec_by_name(require_value(arg));
            if (spec == nullptr) {
                throw std::runtime_error("Unsupported setting name for --get-setting");
            }
            options.control_setting = spec->setting;
        } else if (arg == "--set-setting") {
            options.mode = ProgramMode::ControlClient;
            options.control_command = ControlCommand::SetSetting;
            const auto assignment = require_value(arg);
            const auto equal_pos = assignment.find('=');
            if (equal_pos == std::string::npos) {
                throw std::runtime_error("--set-setting expects name=value");
            }
            const auto* spec = find_setting_spec_by_name(assignment.substr(0, equal_pos));
            if (spec == nullptr) {
                throw std::runtime_error("Unsupported setting name for --set-setting");
            }
            if (spec->shape != SettingValueShape::Scalar || !spec->writable) {
                throw std::runtime_error("Setting is not writable as a scalar value: " + assignment.substr(0, equal_pos));
            }
            options.control_setting = spec->setting;
            options.control_value = parse_int(assignment.substr(equal_pos + 1U), "--set-setting");
        } else if (arg == "--set-setting-range") {
            options.mode = ProgramMode::ControlClient;
            options.control_command = ControlCommand::SetRange;
            const auto assignment = require_value(arg);
            const auto equal_pos = assignment.find('=');
            const auto colon_pos = assignment.find(':', equal_pos == std::string::npos ? 0U : equal_pos + 1U);
            if (equal_pos == std::string::npos || colon_pos == std::string::npos) {
                throw std::runtime_error("--set-setting-range expects name=min:max");
            }
            const auto* spec = find_setting_spec_by_name(assignment.substr(0, equal_pos));
            if (spec == nullptr) {
                throw std::runtime_error("Unsupported setting name for --set-setting-range");
            }
            if (spec->shape != SettingValueShape::Range || !spec->writable) {
                throw std::runtime_error("Setting is not writable as a range value: " + assignment.substr(0, equal_pos));
            }
            options.control_setting = spec->setting;
            options.control_value = parse_int(assignment.substr(equal_pos + 1U, colon_pos - equal_pos - 1U), "--set-setting-range");
            options.control_value_second = parse_int(assignment.substr(colon_pos + 1U), "--set-setting-range");
        } else if (arg == "--reset-settings") {
            options.mode = ProgramMode::ControlClient;
            options.control_command = ControlCommand::ResetSettings;
        } else if (arg == "--list-calibrations") {
            options.mode = ProgramMode::CalibrationClient;
            options.calibration_list_all = true;
        } else if (arg == "--read-calibration") {
            options.mode = ProgramMode::CalibrationClient;
            options.calibration_serial = parse_u32(require_value(arg), arg);
        } else {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    if (options.camera_fps <= 0) {
        throw std::runtime_error("--camera-fps must be positive");
    }
    if (options.service_prefix.empty()) {
        throw std::runtime_error("--service-prefix must not be empty");
    }
    if (options.history_size == 0U) {
        throw std::runtime_error("--history-size must be greater than zero");
    }
    if (options.max_subscribers == 0U) {
        throw std::runtime_error("--max-subscribers must be greater than zero");
    }
    if (options.initial_slice_hint == 0U) {
        throw std::runtime_error("--initial-slice-hint must be greater than zero");
    }

    if (options.mode == ProgramMode::ControlClient
        && options.control_command != ControlCommand::None
        && options.control_selector_kind == CameraSelectorKind::All
        && options.control_command != ControlCommand::ListSettings
        && options.control_command != ControlCommand::ResetSettings) {
        throw std::runtime_error("--camera-target is required for get/set commands");
    }

    if (options.mode == ProgramMode::CalibrationClient
        && !options.calibration_list_all
        && !options.calibration_serial.has_value()) {
        throw std::runtime_error("Calibration client mode requires --list-calibrations or --read-calibration");
    }

    return options;
}

auto create_service_name(const std::string& service_name) -> iox2::bb::Expected<iox2::ServiceName, iox2::bb::SemanticStringError> {
    return iox2::ServiceName::create(service_name.c_str());
}

auto create_ipc_node(const Options& options) -> std::optional<IpcNode> {
    auto node_builder = iox2::NodeBuilder();
    auto node_name_result = iox2::NodeName::create(options.node_name.c_str());
    if (!node_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid node name: " << options.node_name << std::endl;
        return std::nullopt;
    }

    auto node_result = std::move(node_builder).name(std::move(node_name_result.value())).create<iox2::ServiceType::Ipc>();
    if (!node_result.has_value()) {
        const auto failure = node_result.error();
        std::cerr << "[ZED-Iceoryx2] Failed to create iceoryx2 node: "
                  << iox2::bb::into<const char*>(failure) << std::endl;
        return std::nullopt;
    }

    return std::move(node_result.value());
}

auto should_use_camera(const Options& options, const sl::DeviceProperties& device_properties, size_t device_index) -> bool {
    if (!options.selected_serials.empty()) {
        const auto serial_match = std::find(options.selected_serials.begin(),
                                            options.selected_serials.end(),
                                            device_properties.serial_number);
        if (serial_match == options.selected_serials.end()) {
            return false;
        }
    }

    if (!options.selected_indices.empty()) {
        const auto index_match =
            std::find(options.selected_indices.begin(), options.selected_indices.end(), device_index);
        if (index_match == options.selected_indices.end()) {
            return false;
        }
    }

    return true;
}

auto fill_intrinsics(const sl::CameraParameters& src) -> CameraIntrinsics {
    CameraIntrinsics dst {};
    dst.fx = src.fx;
    dst.fy = src.fy;
    dst.cx = src.cx;
    dst.cy = src.cy;
    dst.h_fov_deg = src.h_fov;
    dst.v_fov_deg = src.v_fov;
    dst.d_fov_deg = src.d_fov;
    dst.focal_length_metric = src.focal_length_metric;
    dst.width = static_cast<uint32_t>(src.image_size.width);
    dst.height = static_cast<uint32_t>(src.image_size.height);
    for (size_t i = 0U; i < dst.disto.size(); ++i) {
        dst.disto[i] = src.disto[i];
    }
    return dst;
}

auto fill_transform_matrix(const sl::Transform& transform) -> std::array<float, 16U> {
    std::array<float, 16U> matrix {};
    const float* data = transform.m;
    for (size_t i = 0U; i < matrix.size(); ++i) {
        matrix[i] = data[i];
    }
    return matrix;
}

auto build_calibration_snapshot(const CameraContext& camera_context, const Options& options) -> StereoCalibrationSnapshot {
    const auto requested_resolution =
        options.publish_resolution.has_value()
            ? options.publish_resolution.value()
            : camera_context.camera.getCameraInformation().camera_configuration.resolution;
    const auto info = camera_context.camera.getCameraInformation(requested_resolution);

    StereoCalibrationSnapshot snapshot {};
    snapshot.serial_number = camera_context.serial_number;
    snapshot.camera_index = static_cast<uint32_t>(camera_context.device_index);
    snapshot.camera_model = static_cast<uint32_t>(camera_context.camera_model);
    snapshot.firmware_version = info.camera_configuration.firmware_version;
    snapshot.sensors_firmware_version = info.sensors_configuration.firmware_version;
    snapshot.active_width = static_cast<uint32_t>(requested_resolution.width);
    snapshot.active_height = static_cast<uint32_t>(requested_resolution.height);
    snapshot.crop_offset_x = 0U;
    snapshot.crop_offset_y = 0U;
    snapshot.crop_width = snapshot.active_width;
    snapshot.crop_height = snapshot.active_height;
    snapshot.is_rectified_stream = options.rectified ? 1U : 0U;
    snapshot.has_publish_resolution_override = options.publish_resolution.has_value() ? 1U : 0U;
    snapshot.fps = info.camera_configuration.fps;

    const auto& rectified = info.camera_configuration.calibration_parameters;
    const auto& raw = info.camera_configuration.calibration_parameters_raw;

    snapshot.rectified_baseline = rectified.getCameraBaseline();
    snapshot.raw_baseline = raw.getCameraBaseline();
    snapshot.rectified_stereo_transform = fill_transform_matrix(rectified.stereo_transform);
    snapshot.raw_stereo_transform = fill_transform_matrix(raw.stereo_transform);
    snapshot.rectified_left = fill_intrinsics(rectified.left_cam);
    snapshot.rectified_right = fill_intrinsics(rectified.right_cam);
    snapshot.raw_left = fill_intrinsics(raw.left_cam);
    snapshot.raw_right = fill_intrinsics(raw.right_cam);

    return snapshot;
}

auto create_stream_endpoint(const IpcNode& node,
                            const Options& options,
                            const std::string& service_name,
                            sl::VIEW view) -> std::optional<StreamEndpoint> {
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto service_result = node.service_builder(service_name_result.value())
                              .publish_subscribe<BytePayload>()
                              .user_header<FrameHeader>()
                              .history_size(options.history_size)
                              .max_publishers(1)
                              .max_subscribers(options.max_subscribers)
                              .enable_safe_overflow(true)
                              .open_or_create();

    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to open or create service: " << service_name << std::endl;
        return std::nullopt;
    }

    StreamEndpoint endpoint;
    endpoint.service_name = service_name;
    endpoint.view = view;
    endpoint.service = std::move(service_result.value());

    auto publisher_result = endpoint.service->publisher_builder()
                                .initial_max_slice_len(options.initial_slice_hint)
                                .allocation_strategy(iox2::AllocationStrategy::PowerOfTwo)
                                .create();

    if (!publisher_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create publisher: " << service_name << std::endl;
        return std::nullopt;
    }

    endpoint.publisher = std::move(publisher_result.value());
    return endpoint;
}

auto create_control_channel(const IpcNode& node, const Options& options) -> std::optional<ControlChannel> {
    const auto service_name = options.service_prefix + "/control";
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid control service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto service_result = node.service_builder(service_name_result.value())
                              .request_response<ControlRequest, ControlResponse>()
                              .max_servers(1)
                              .max_clients(16)
                              .max_nodes(32)
                              .max_active_requests_per_client(8)
                              .max_response_buffer_size(256)
                              .max_borrowed_responses_per_pending_response(256)
                              .max_loaned_requests(4)
                              .enable_safe_overflow_for_requests(true)
                              .enable_safe_overflow_for_responses(true)
                              .open_or_create();
    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to open or create control service: " << service_name << std::endl;
        return std::nullopt;
    }

    ControlChannel channel;
    channel.service_name = service_name;
    channel.service = std::move(service_result.value());

    auto server_result = channel.service->server_builder().create();
    if (!server_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create control server: " << service_name << std::endl;
        return std::nullopt;
    }

    channel.server = std::move(server_result.value());
    return channel;
}

auto create_calibration_channel(const IpcNode& node,
                                const Options& options,
                                const std::vector<std::unique_ptr<CameraContext>>& cameras) -> std::optional<CalibrationChannel> {
    const auto service_name = options.service_prefix + "/calibration";
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid calibration service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto builder = node.service_builder(service_name_result.value()).blackboard_creator<uint32_t>().max_readers(32).max_nodes(32);
    for (const auto& camera_context : cameras) {
        builder.add_with_default<StereoCalibrationSnapshot>(camera_context->serial_number);
    }

    auto service_result = std::move(builder).create();
    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create calibration blackboard: " << service_name << std::endl;
        return std::nullopt;
    }

    CalibrationChannel channel;
    channel.service_name = service_name;
    channel.service = std::move(service_result.value());

    auto writer_result = channel.service->writer_builder().create();
    if (!writer_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create calibration writer: " << service_name << std::endl;
        return std::nullopt;
    }

    channel.writer = std::move(writer_result.value());
    return channel;
}

auto retrieve_image(sl::Camera& camera,
                    sl::Mat& image,
                    sl::VIEW view,
                    const std::optional<sl::Resolution>& publish_resolution) -> sl::ERROR_CODE {
    if (publish_resolution.has_value()) {
        return camera.retrieveImage(image, view, sl::MEM::CPU, publish_resolution.value());
    }

    return camera.retrieveImage(image, view, sl::MEM::CPU);
}

auto publish_image(CameraContext& camera_context,
                   StreamEndpoint& endpoint,
                   sl::Mat& image,
                   uint64_t timestamp_ns) -> bool {
    if (!endpoint.publisher.has_value()) {
        return false;
    }

    auto* image_ptr = image.getPtr<sl::uchar1>(sl::MEM::CPU);
    const auto image_bytes = image.getStepBytes(sl::MEM::CPU) * image.getHeight();
    if (image_ptr == nullptr || image_bytes == 0U) {
        std::cerr << "[ZED-Iceoryx2] Empty frame for service " << endpoint.service_name << std::endl;
        return false;
    }

    auto loan_result = endpoint.publisher->loan_slice_uninit(image_bytes);
    if (!loan_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to loan payload for " << endpoint.service_name << std::endl;
        return false;
    }

    FrameHeader header {};
    header.timestamp_ns = timestamp_ns;
    header.frame_id = camera_context.frame_count;
    header.serial_number = camera_context.serial_number;
    header.camera_index = static_cast<uint32_t>(camera_context.device_index);
    header.camera_model = static_cast<uint32_t>(camera_context.camera_model);
    header.view = static_cast<uint32_t>(endpoint.view);
    header.width = static_cast<uint32_t>(image.getWidth());
    header.height = static_cast<uint32_t>(image.getHeight());
    header.step_bytes = static_cast<uint32_t>(image.getStepBytes(sl::MEM::CPU));
    header.image_bytes = static_cast<uint32_t>(image_bytes);
    header.channels = static_cast<uint32_t>(image.getChannels());
    header.bytes_per_pixel = static_cast<uint32_t>(image.getPixelBytes());
    header.mat_type = static_cast<uint32_t>(image.getDataType());
    header.reserved = 0U;

    loan_result->user_header_mut() = header;

    iox2::bb::ImmutableSlice<uint8_t> image_slice(reinterpret_cast<const uint8_t*>(image_ptr), image_bytes);
    auto initialized_sample = loan_result->write_from_slice(image_slice);
    auto send_result = iox2::send(std::move(initialized_sample));
    if (!send_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to send frame for " << endpoint.service_name << std::endl;
        return false;
    }

    ++camera_context.published_frames;
    return true;
}

auto create_camera_context(const IpcNode& node,
                           const Options& options,
                           const sl::DeviceProperties& device_properties,
                           size_t device_index) -> std::unique_ptr<CameraContext> {
    auto context = std::make_unique<CameraContext>();
    context->device_index = device_index;
    context->serial_number = device_properties.serial_number;
    context->camera_model = device_properties.camera_model;

    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = options.camera_resolution;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.camera_fps = options.camera_fps;
    init_parameters.input.setFromSerialNumber(device_properties.serial_number);

    const auto open_result = context->camera.open(init_parameters);
    if (open_result != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[ZED-Iceoryx2] Failed to open camera " << device_index << ", Error: " << open_result << std::endl;
        return nullptr;
    }

    const auto stream_base = options.service_prefix + "/zed" + std::to_string(device_index + 1U);
    const auto left_view = options.rectified ? sl::VIEW::LEFT : sl::VIEW::LEFT_UNRECTIFIED;
    const auto right_view = options.rectified ? sl::VIEW::RIGHT : sl::VIEW::RIGHT_UNRECTIFIED;

    auto left_endpoint = create_stream_endpoint(node, options, stream_base + "/left", left_view);
    auto right_endpoint = create_stream_endpoint(node, options, stream_base + "/right", right_view);

    if (!left_endpoint.has_value() && !right_endpoint.has_value()) {
        std::cerr << "[ZED-Iceoryx2] No publishers created for camera " << device_index << std::endl;
        context->camera.close();
        return nullptr;
    }

    if (left_endpoint.has_value()) {
        context->left = std::move(left_endpoint.value());
    }
    if (right_endpoint.has_value()) {
        context->right = std::move(right_endpoint.value());
    }

    context->calibration = build_calibration_snapshot(*context, options);

    const auto camera_info = context->camera.getCameraInformation();
    std::cout << "[ZED-Iceoryx2] Opened camera " << device_index
              << " serial=" << camera_info.serial_number
              << " left_service=" << context->left.service_name
              << " right_service=" << context->right.service_name << std::endl;

    return context;
}

auto write_calibration_snapshot(CalibrationChannel& channel, const CameraContext& camera_context) -> bool {
    if (!channel.writer.has_value()) {
        return false;
    }

    auto entry_result = channel.writer->entry<StereoCalibrationSnapshot>(camera_context.serial_number);
    if (!entry_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to acquire calibration entry for serial "
                  << camera_context.serial_number << std::endl;
        return false;
    }

    entry_result->update_with_copy(camera_context.calibration);
    return true;
}

auto camera_matches_selector(const CameraContext& camera_context, const ControlRequest& request) -> bool {
    const auto selector_kind = static_cast<CameraSelectorKind>(request.selector_kind);
    switch (selector_kind) {
        case CameraSelectorKind::All:
            return true;
        case CameraSelectorKind::Serial:
            return camera_context.serial_number == request.selector_value;
        case CameraSelectorKind::Index:
            return static_cast<uint32_t>(camera_context.device_index) == request.selector_value;
    }
    return false;
}

auto make_response_base(const CameraContext& camera_context, const ControlRequest& request) -> ControlResponse {
    ControlResponse response {};
    response.command = request.command;
    response.serial_number = camera_context.serial_number;
    response.camera_index = static_cast<uint32_t>(camera_context.device_index);
    response.camera_model = static_cast<uint32_t>(camera_context.camera_model);
    response.setting = request.setting;
    return response;
}

auto store_c_string(const std::string& value, char* destination, size_t destination_size) -> void {
    if (destination_size == 0U) {
        return;
    }
    std::snprintf(destination, destination_size, "%s", value.c_str());
}

auto respond_with_error(iox2::ActiveRequest<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>& active_request,
                        const ControlResponse& response) -> void {
    auto send_result = active_request.send_copy(response);
    if (!send_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to send control error response" << std::endl;
    }
}

auto send_setting_snapshot(iox2::ActiveRequest<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>& active_request,
                           CameraContext& camera_context,
                           const SettingSpec& spec,
                           const std::string& message) -> bool {
    ControlResponse response = make_response_base(camera_context, active_request.payload());
    response.setting = static_cast<uint32_t>(spec.setting);
    response.shape = static_cast<uint32_t>(spec.shape);
    store_c_string(spec.name, response.setting_name, sizeof(response.setting_name));
    store_c_string(message, response.message, sizeof(response.message));

    sl::ERROR_CODE zed_result = sl::ERROR_CODE::SUCCESS;
    if (spec.shape == SettingValueShape::Range) {
        int min_value = 0;
        int max_value = 0;
        zed_result = camera_context.camera.getCameraSettings(spec.setting, min_value, max_value);
        response.value = min_value;
        response.value_second = max_value;
    } else {
        int value = 0;
        zed_result = camera_context.camera.getCameraSettings(spec.setting, value);
        response.value = value;
    }

    int range_min = 0;
    int range_max = 0;
    const auto range_result = camera_context.camera.getCameraSettingsRange(spec.setting, range_min, range_max);
    if (range_result == sl::ERROR_CODE::SUCCESS) {
        response.min_value = range_min;
        response.max_value = range_max;
    }

    if (zed_result != sl::ERROR_CODE::SUCCESS) {
        response.status = static_cast<uint32_t>(ControlStatus::ZedError);
        response.zed_error_code = static_cast<int32_t>(zed_result);
        store_c_string("Failed to query setting", response.message, sizeof(response.message));
    }

    auto send_result = active_request.send_copy(response);
    if (!send_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to send control response for camera "
                  << camera_context.serial_number << std::endl;
        return false;
    }
    return true;
}

auto handle_control_request(
    iox2::ActiveRequest<iox2::ServiceType::Ipc, ControlRequest, void, ControlResponse, void>& active_request,
    std::vector<std::unique_ptr<CameraContext>>& cameras,
    const Options& options,
    CalibrationChannel* calibration_channel) -> void {
    const auto request = active_request.payload();
    const auto command = static_cast<ControlCommand>(request.command);

    std::vector<CameraContext*> matches;
    for (auto& camera_context : cameras) {
        if (camera_matches_selector(*camera_context, request)) {
            matches.push_back(camera_context.get());
        }
    }

    if (matches.empty()) {
        ControlResponse response {};
        response.status = static_cast<uint32_t>(ControlStatus::CameraNotFound);
        response.command = request.command;
        response.setting = request.setting;
        store_c_string("No matching camera for request", response.message, sizeof(response.message));
        respond_with_error(active_request, response);
        return;
    }

    if (command == ControlCommand::ListSettings) {
        for (auto* camera_context : matches) {
            for (const auto& spec : kSettingSpecs) {
                send_setting_snapshot(active_request, *camera_context, spec, "setting");
            }
        }
        return;
    }

    if ((command == ControlCommand::GetSetting || command == ControlCommand::SetSetting || command == ControlCommand::SetRange)
        && request.setting == INVALID_SETTING_ID) {
        ControlResponse response {};
        response.status = static_cast<uint32_t>(ControlStatus::InvalidRequest);
        response.command = request.command;
        store_c_string("Request did not include a setting", response.message, sizeof(response.message));
        respond_with_error(active_request, response);
        return;
    }

    const auto setting = static_cast<sl::VIDEO_SETTINGS>(request.setting);
    const auto* spec = find_setting_spec(setting);

    if ((command == ControlCommand::GetSetting || command == ControlCommand::SetSetting || command == ControlCommand::SetRange)
        && spec == nullptr) {
        ControlResponse response {};
        response.status = static_cast<uint32_t>(ControlStatus::UnsupportedSetting);
        response.command = request.command;
        response.setting = request.setting;
        store_c_string("Unknown or unsupported setting", response.message, sizeof(response.message));
        respond_with_error(active_request, response);
        return;
    }

    for (auto* camera_context : matches) {
        ControlResponse response = make_response_base(*camera_context, request);

        switch (command) {
            case ControlCommand::GetSetting:
                send_setting_snapshot(active_request, *camera_context, *spec, "setting");
                break;

            case ControlCommand::SetSetting:
                if (!spec->writable || spec->shape != SettingValueShape::Scalar) {
                    response.status = static_cast<uint32_t>(ControlStatus::UnsupportedSetting);
                    store_c_string("Setting is not writable as a scalar", response.message, sizeof(response.message));
                } else {
                    const auto zed_result = camera_context->camera.setCameraSettings(spec->setting, request.value);
                    response.zed_error_code = static_cast<int32_t>(zed_result);
                    if (zed_result != sl::ERROR_CODE::SUCCESS) {
                        response.status = static_cast<uint32_t>(ControlStatus::ZedError);
                        store_c_string("Failed to apply scalar setting", response.message, sizeof(response.message));
                    } else {
                        store_c_string("Applied scalar setting", response.message, sizeof(response.message));
                    }
                    store_c_string(spec->name, response.setting_name, sizeof(response.setting_name));
                }
                if (response.status == static_cast<uint32_t>(ControlStatus::Ok)) {
                    send_setting_snapshot(active_request, *camera_context, *spec, "applied");
                } else {
                    respond_with_error(active_request, response);
                }
                break;

            case ControlCommand::SetRange:
                if (!spec->writable || spec->shape != SettingValueShape::Range) {
                    response.status = static_cast<uint32_t>(ControlStatus::UnsupportedSetting);
                    store_c_string("Setting is not writable as a range", response.message, sizeof(response.message));
                } else {
                    const auto zed_result =
                        camera_context->camera.setCameraSettings(spec->setting, request.value, request.value_second);
                    response.zed_error_code = static_cast<int32_t>(zed_result);
                    if (zed_result != sl::ERROR_CODE::SUCCESS) {
                        response.status = static_cast<uint32_t>(ControlStatus::ZedError);
                        store_c_string("Failed to apply range setting", response.message, sizeof(response.message));
                    } else {
                        store_c_string("Applied range setting", response.message, sizeof(response.message));
                    }
                    store_c_string(spec->name, response.setting_name, sizeof(response.setting_name));
                }
                if (response.status == static_cast<uint32_t>(ControlStatus::Ok)) {
                    send_setting_snapshot(active_request, *camera_context, *spec, "applied");
                } else {
                    respond_with_error(active_request, response);
                }
                break;

            case ControlCommand::ResetSettings:
                {
                    sl::ERROR_CODE last_error = sl::ERROR_CODE::SUCCESS;
                    for (const auto& reset_spec : kSettingSpecs) {
                        if (!reset_spec.writable) {
                            continue;
                        }
                        const auto reset_result =
                            camera_context->camera.setCameraSettings(reset_spec.setting, sl::VIDEO_SETTINGS_VALUE_AUTO);
                        if (reset_result != sl::ERROR_CODE::SUCCESS) {
                            last_error = reset_result;
                        }
                    }
                    response.zed_error_code = static_cast<int32_t>(last_error);
                    if (last_error != sl::ERROR_CODE::SUCCESS) {
                        response.status = static_cast<uint32_t>(ControlStatus::ZedError);
                        store_c_string("Some settings failed to reset", response.message, sizeof(response.message));
                    } else {
                        store_c_string("Reset supported settings to defaults", response.message, sizeof(response.message));
                    }
                    respond_with_error(active_request, response);
                }
                break;

            case ControlCommand::None:
                response.status = static_cast<uint32_t>(ControlStatus::InvalidRequest);
                store_c_string("No command specified", response.message, sizeof(response.message));
                respond_with_error(active_request, response);
                break;
        }

        camera_context->calibration = build_calibration_snapshot(*camera_context, options);
        if (calibration_channel != nullptr) {
            write_calibration_snapshot(*calibration_channel, *camera_context);
        }
    }
}

auto print_device_properties_summary(const sl::DeviceProperties& device_properties, size_t index, bool selected) -> void {
    std::cout << "[ZED-Iceoryx2] Camera " << index
              << (selected ? " [selected]" : " [filtered]")
              << ": model=" << device_properties.camera_model
              << ", state=" << device_properties.camera_state
              << ", serial=" << device_properties.serial_number
              << ", path=" << device_properties.path.c_str()
              << ", video_device=" << device_properties.video_device.c_str()
              << ", badge=" << device_properties.camera_badge.c_str()
              << ", sensor=" << device_properties.camera_sensor_model.c_str()
              << ", name=" << device_properties.camera_name.c_str()
              << ", gmsl_port=" << device_properties.gmsl_port
              << ", i2c_port=" << device_properties.i2c_port
              << std::endl;
}

auto list_cameras(const Options& options) -> int {
    const auto device_list = sl::Camera::getDeviceList();
    std::cout << "[ZED-Iceoryx2] Found " << device_list.size() << " ZED camera(s)" << std::endl;

    for (size_t i = 0U; i < device_list.size(); ++i) {
        const auto selected = should_use_camera(options, device_list[i], i);
        print_device_properties_summary(device_list[i], i, selected);

        sl::Camera camera;
        sl::InitParameters init_parameters;
        init_parameters.camera_resolution = options.camera_resolution;
        init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
        init_parameters.camera_fps = options.camera_fps;
        init_parameters.input.setFromSerialNumber(device_list[i].serial_number);

        const auto open_result = camera.open(init_parameters);
        if (open_result != sl::ERROR_CODE::SUCCESS) {
            std::cout << "  open_status=" << open_result << std::endl;
            continue;
        }

        const auto camera_info = camera.getCameraInformation();
        std::cout << "  firmware=" << camera_info.camera_configuration.firmware_version
                  << ", sensors_firmware=" << camera_info.sensors_configuration.firmware_version
                  << ", resolution=" << camera_info.camera_configuration.resolution.width
                  << "x" << camera_info.camera_configuration.resolution.height
                  << ", fps=" << camera_info.camera_configuration.fps
                  << ", input_type=" << camera_info.input_type
                  << std::endl;
        camera.close();
    }

    return EXIT_SUCCESS;
}

auto print_control_response(const ControlResponse& response) -> void {
    std::cout << "[ZED-Iceoryx2] camera_index=" << response.camera_index
              << " serial=" << response.serial_number
              << " model=" << response.camera_model
              << " status=" << response.status;
    if (response.setting != INVALID_SETTING_ID) {
        std::cout << " setting=" << response.setting_name;
    }
    if (static_cast<SettingValueShape>(response.shape) == SettingValueShape::Range) {
        std::cout << " value=" << response.value << ":" << response.value_second;
    } else {
        std::cout << " value=" << response.value;
    }
    if (response.min_value != 0 || response.max_value != 0) {
        std::cout << " allowed_range=" << response.min_value << ":" << response.max_value;
    }
    if (response.zed_error_code != 0) {
        std::cout << " zed_error=" << response.zed_error_code;
    }
    if (response.message[0] != '\0') {
        std::cout << " message=\"" << response.message << "\"";
    }
    std::cout << std::endl;
}

auto open_control_client(const IpcNode& node, const Options& options) -> std::optional<ControlClient> {
    const auto service_name = options.service_prefix + "/control";
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid control service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto service_result = node.service_builder(service_name_result.value())
                              .request_response<ControlRequest, ControlResponse>()
                              .open();
    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to open control service: " << service_name << std::endl;
        return std::nullopt;
    }

    auto client_result = service_result->client_builder().create();
    if (!client_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create control client: " << service_name << std::endl;
        return std::nullopt;
    }

    return std::move(client_result.value());
}

auto run_control_client(const Options& options) -> int {
    auto node = create_ipc_node(options);
    if (!node.has_value()) {
        return EXIT_FAILURE;
    }

    auto client = open_control_client(node.value(), options);
    if (!client.has_value()) {
        return EXIT_FAILURE;
    }

    ControlRequest request {};
    request.command = static_cast<uint32_t>(options.control_command);
    request.selector_kind = static_cast<uint32_t>(options.control_selector_kind);
    request.selector_value = options.control_selector_value;
    request.setting = options.control_setting.has_value() ? static_cast<uint32_t>(options.control_setting.value()) : INVALID_SETTING_ID;
    request.value = options.control_value;
    request.value_second = options.control_value_second;

    auto pending_response_result = client->send_copy(request);
    if (!pending_response_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to send control request" << std::endl;
        return EXIT_FAILURE;
    }

    auto pending_response = std::move(pending_response_result.value());
    auto deadline = iox2::bb::Duration::from_millis(200);
    size_t responses = 0U;
    size_t idle_loops = 0U;

    while (pending_response.is_connected() || pending_response.has_response()) {
        auto receive_result = pending_response.receive();
        if (!receive_result.has_value()) {
            std::cerr << "[ZED-Iceoryx2] Failed to receive control response" << std::endl;
            return EXIT_FAILURE;
        }

        if (receive_result->has_value()) {
            print_control_response(receive_result->value().payload());
            ++responses;
            idle_loops = 0U;
        } else {
            ++idle_loops;
            if (idle_loops > 25U) {
                break;
            }
            static_cast<void>(node->wait(deadline));
        }
    }

    if (responses == 0U) {
        std::cerr << "[ZED-Iceoryx2] No control responses received" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

auto open_calibration_reader(const IpcNode& node, const Options& options) -> std::optional<CalibrationReader> {
    const auto service_name = options.service_prefix + "/calibration";
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid calibration service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto service_result = node.service_builder(service_name_result.value()).blackboard_opener<uint32_t>().open();
    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to open calibration blackboard: " << service_name << std::endl;
        return std::nullopt;
    }

    auto reader_result = service_result->reader_builder().create();
    if (!reader_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create calibration reader: " << service_name << std::endl;
        return std::nullopt;
    }

    return std::move(reader_result.value());
}

auto print_intrinsics(const char* label, const CameraIntrinsics& intrinsics) -> void {
    std::cout << "  " << label
              << ": size=" << intrinsics.width << "x" << intrinsics.height
              << " fx=" << intrinsics.fx
              << " fy=" << intrinsics.fy
              << " cx=" << intrinsics.cx
              << " cy=" << intrinsics.cy
              << " hfov=" << intrinsics.h_fov_deg
              << " vfov=" << intrinsics.v_fov_deg
              << std::endl;
}

auto print_calibration_snapshot(const StereoCalibrationSnapshot& snapshot) -> void {
    std::cout << "[ZED-Iceoryx2] serial=" << snapshot.serial_number
              << " camera_index=" << snapshot.camera_index
              << " model=" << snapshot.camera_model
              << " active_resolution=" << snapshot.active_width << "x" << snapshot.active_height
              << " rectified_stream=" << (snapshot.is_rectified_stream != 0U ? "true" : "false")
              << " fps=" << snapshot.fps
              << " firmware=" << snapshot.firmware_version
              << " sensors_firmware=" << snapshot.sensors_firmware_version
              << std::endl;
    print_intrinsics("rectified_left", snapshot.rectified_left);
    print_intrinsics("rectified_right", snapshot.rectified_right);
    print_intrinsics("raw_left", snapshot.raw_left);
    print_intrinsics("raw_right", snapshot.raw_right);
    std::cout << "  rectified_baseline=" << snapshot.rectified_baseline
              << " raw_baseline=" << snapshot.raw_baseline
              << std::endl;
}

auto run_calibration_client(const Options& options) -> int {
    auto node = create_ipc_node(options);
    if (!node.has_value()) {
        return EXIT_FAILURE;
    }

    const auto service_name = options.service_prefix + "/calibration";
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid calibration service name: " << service_name << std::endl;
        return EXIT_FAILURE;
    }

    auto service_result = node->service_builder(service_name_result.value()).blackboard_opener<uint32_t>().open();
    if (!service_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to open calibration blackboard: " << service_name << std::endl;
        return EXIT_FAILURE;
    }

    auto reader_result = service_result->reader_builder().create();
    if (!reader_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to create calibration reader: " << service_name << std::endl;
        return EXIT_FAILURE;
    }
    auto reader = std::move(reader_result.value());

    if (options.calibration_list_all) {
        size_t count = 0U;
        service_result->list_keys([&](const uint32_t& key) {
            auto entry_result = reader.entry<StereoCalibrationSnapshot>(key);
            if (entry_result.has_value()) {
                auto snapshot = entry_result->get();
                print_calibration_snapshot(*snapshot);
                ++count;
            }
            return iox2::CallbackProgression::Continue;
        });
        if (count == 0U) {
            std::cerr << "[ZED-Iceoryx2] No calibration snapshots available" << std::endl;
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }

    auto entry_result = reader.entry<StereoCalibrationSnapshot>(options.calibration_serial.value());
    if (!entry_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Calibration snapshot not found for serial "
                  << options.calibration_serial.value() << std::endl;
        return EXIT_FAILURE;
    }

    auto snapshot = entry_result->get();
    print_calibration_snapshot(*snapshot);
    return EXIT_SUCCESS;
}

auto run_streamer(const Options& options) -> int {
    iox2::set_log_level_from_env_or(iox2::LogLevel::Info);

    auto node = create_ipc_node(options);
    if (!node.has_value()) {
        return EXIT_FAILURE;
    }

    const auto device_list = sl::Camera::getDeviceList();
    std::cout << "[ZED-Iceoryx2] Found " << device_list.size() << " ZED camera(s)" << std::endl;

    if (device_list.empty()) {
        std::cerr << "[ZED-Iceoryx2] No ZED cameras detected. Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<std::unique_ptr<CameraContext>> cameras;
    cameras.reserve(device_list.size());

    for (size_t i = 0U; i < device_list.size(); ++i) {
        const auto selected = should_use_camera(options, device_list[i], i);
        print_device_properties_summary(device_list[i], i, selected);
        if (!selected) {
            continue;
        }

        auto context = create_camera_context(node.value(), options, device_list[i], i);
        if (context != nullptr) {
            cameras.push_back(std::move(context));
        }
    }

    if (cameras.empty()) {
        std::cerr << "[ZED-Iceoryx2] No cameras opened successfully. Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    auto control_channel = create_control_channel(node.value(), options);
    if (!control_channel.has_value()) {
        return EXIT_FAILURE;
    }

    auto calibration_channel = create_calibration_channel(node.value(), options, cameras);
    if (!calibration_channel.has_value()) {
        return EXIT_FAILURE;
    }

    for (auto& camera_context : cameras) {
        camera_context->calibration = build_calibration_snapshot(*camera_context, options);
        write_calibration_snapshot(calibration_channel.value(), *camera_context);
    }

    sl::RuntimeParameters runtime_parameters;
    const auto cycle_time = iox2::bb::Duration::from_millis(std::max(1, 1000 / std::max(1, options.camera_fps)));

    std::cout << "[ZED-Iceoryx2] Publishing with service prefix '" << options.service_prefix << "'"
              << ", camera_fps=" << options.camera_fps
              << ", rectified=" << (options.rectified ? "true" : "false");
    if (options.publish_resolution.has_value()) {
        std::cout << ", publish_resolution=" << options.publish_resolution->width
                  << "x" << options.publish_resolution->height;
    }
    std::cout << std::endl;
    std::cout << "[ZED-Iceoryx2] Control service: " << control_channel->service_name << std::endl;
    std::cout << "[ZED-Iceoryx2] Calibration blackboard: " << calibration_channel->service_name << std::endl;
    std::cout << "[ZED-Iceoryx2] Press Ctrl+C to stop" << std::endl;

    uint64_t iteration = 0U;
    while (node->wait(cycle_time).has_value()) {
        if (control_channel->server.has_value()) {
            while (control_channel->server->has_requests().has_value() && control_channel->server->has_requests().value()) {
                auto receive_result = control_channel->server->receive();
                if (!receive_result.has_value()) {
                    std::cerr << "[ZED-Iceoryx2] Failed to receive control request" << std::endl;
                    break;
                }
                if (receive_result->has_value()) {
                    auto active_request = std::move(receive_result->value());
                    handle_control_request(active_request, cameras, options, &calibration_channel.value());
                } else {
                    break;
                }
            }
        }

        for (auto& camera_context : cameras) {
            if (camera_context->camera.grab(runtime_parameters) != sl::ERROR_CODE::SUCCESS) {
                continue;
            }

            ++camera_context->frame_count;
            const auto timestamp_ns = camera_context->camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns;

            if (camera_context->left.publisher.has_value()) {
                sl::Mat left_image;
                if (retrieve_image(camera_context->camera,
                                   left_image,
                                   camera_context->left.view,
                                   options.publish_resolution)
                    == sl::ERROR_CODE::SUCCESS) {
                    publish_image(*camera_context, camera_context->left, left_image, timestamp_ns);
                }
            }

            if (camera_context->right.publisher.has_value()) {
                sl::Mat right_image;
                if (retrieve_image(camera_context->camera,
                                   right_image,
                                   camera_context->right.view,
                                   options.publish_resolution)
                    == sl::ERROR_CODE::SUCCESS) {
                    publish_image(*camera_context, camera_context->right, right_image, timestamp_ns);
                }
            }
        }

        ++iteration;
        if (iteration % 100U == 0U) {
            std::cout << "[ZED-Iceoryx2] iteration=" << iteration;
            for (const auto& camera_context : cameras) {
                std::cout << " camera" << camera_context->device_index
                          << "_grabbed=" << camera_context->frame_count
                          << " camera" << camera_context->device_index
                          << "_published=" << camera_context->published_frames;
            }
            std::cout << std::endl;
        }
    }

    std::cout << "[ZED-Iceoryx2] Shutting down..." << std::endl;
    for (auto& camera_context : cameras) {
        camera_context->camera.close();
    }
    std::cout << "[ZED-Iceoryx2] Shutdown complete" << std::endl;
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse_options(argc, argv);

        switch (options.mode) {
            case ProgramMode::ListCameras:
                return list_cameras(options);
            case ProgramMode::ControlClient:
                return run_control_client(options);
            case ProgramMode::CalibrationClient:
                return run_calibration_client(options);
            case ProgramMode::Stream:
                return run_streamer(options);
        }

        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "[ZED-Iceoryx2] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
