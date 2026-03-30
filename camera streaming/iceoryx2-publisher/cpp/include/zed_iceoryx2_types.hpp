#pragma once

#include <sl/Camera.hpp>

#include "iox2/iceoryx2.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

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
    sl::RESOLUTION camera_resolution = sl::RESOLUTION::HD720;
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

inline constexpr std::array<SettingSpec, 16U> kSettingSpecs {{
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
