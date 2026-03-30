#pragma once

#include "zed_iceoryx2_types.hpp"

#include <memory>
#include <optional>
#include <string>

auto create_service_name(const std::string& service_name) -> iox2::bb::Expected<iox2::ServiceName, iox2::bb::SemanticStringError>;
auto make_camera_service_base(const Options& options, size_t device_index) -> std::string;

class CameraInstanceService {
public:
    static auto create(const IpcNode& node,
                       const Options& options,
                       const sl::DeviceProperties& device_properties,
                       size_t device_index) -> std::unique_ptr<CameraInstanceService>;

    auto deviceIndex() const -> size_t;
    auto serialNumber() const -> uint32_t;
    auto cameraModel() const -> sl::MODEL;
    auto instanceName() const -> const std::string&;
    auto frameCount() const -> uint64_t;
    auto publishedFrames() const -> uint64_t;
    auto leftServiceName() const -> const std::string&;
    auto rightServiceName() const -> const std::string&;
    auto controlServiceName() const -> const std::string&;
    auto calibrationServiceName() const -> const std::string&;

    auto camera() -> sl::Camera&;
    auto controlChannel() -> std::optional<ControlChannel>&;

    auto matchesSelector(const ControlRequest& request) const -> bool;
    auto makeResponseBase(const ControlRequest& request) const -> ControlResponse;

    auto refreshCalibration(const Options& options, CalibrationChannel* aggregate_channel) -> void;
    auto publishCurrentFrames(const Options& options, const sl::RuntimeParameters& runtime_parameters) -> void;

private:
    size_t device_index_ = 0U;
    uint32_t serial_number_ = 0U;
    sl::MODEL camera_model_ = sl::MODEL::ZED;
    sl::Camera camera_ {};
    uint64_t frame_count_ = 0U;
    uint64_t published_frames_ = 0U;
    std::string instance_name_;
    StreamEndpoint left_ {};
    StreamEndpoint right_ {};
    std::optional<ControlChannel> control_ {};
    std::optional<CalibrationChannel> calibration_channel_ {};
    StereoCalibrationSnapshot calibration_ {};

    auto buildCalibrationSnapshot(const Options& options) const -> StereoCalibrationSnapshot;
    auto writeCalibrationSnapshot(CalibrationChannel& channel) const -> bool;
    auto writeCalibrationSnapshots(CalibrationChannel* aggregate_channel) -> void;
    auto publishImage(StreamEndpoint& endpoint, sl::Mat& image, uint64_t timestamp_ns) -> bool;
};
