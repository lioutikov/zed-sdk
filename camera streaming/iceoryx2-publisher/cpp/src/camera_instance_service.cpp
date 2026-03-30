#include "camera_instance_service.hpp"

#include <cstdio>
#include <iostream>
#include <utility>

namespace {

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

auto create_control_channel(const IpcNode& node, const std::string& service_name) -> std::optional<ControlChannel> {
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
                                const std::string& service_name,
                                uint32_t serial_number) -> std::optional<CalibrationChannel> {
    auto service_name_result = create_service_name(service_name);
    if (!service_name_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Invalid calibration service name: " << service_name << std::endl;
        return std::nullopt;
    }

    auto builder = node.service_builder(service_name_result.value()).blackboard_creator<uint32_t>().max_readers(32).max_nodes(32);
    builder.add_with_default<StereoCalibrationSnapshot>(serial_number);

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

} // namespace

auto create_service_name(const std::string& service_name) -> iox2::bb::Expected<iox2::ServiceName, iox2::bb::SemanticStringError> {
    return iox2::ServiceName::create(service_name.c_str());
}

auto make_camera_service_base(const Options& options, size_t device_index) -> std::string {
    return options.service_prefix + "/zed" + std::to_string(device_index + 1U);
}

auto CameraInstanceService::create(const IpcNode& node,
                                   const Options& options,
                                   const sl::DeviceProperties& device_properties,
                                   size_t device_index) -> std::unique_ptr<CameraInstanceService> {
    auto instance = std::make_unique<CameraInstanceService>();
    instance->device_index_ = device_index;
    instance->serial_number_ = device_properties.serial_number;
    instance->camera_model_ = device_properties.camera_model;
    instance->instance_name_ = "zed" + std::to_string(device_index + 1U);

    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = options.camera_resolution;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.camera_fps = options.camera_fps;
    init_parameters.input.setFromSerialNumber(device_properties.serial_number);

    const auto open_result = instance->camera_.open(init_parameters);
    if (open_result != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[ZED-Iceoryx2] Failed to open camera " << device_index << ", Error: " << open_result << std::endl;
        return nullptr;
    }

    const auto stream_base = make_camera_service_base(options, device_index);
    const auto left_view = options.rectified ? sl::VIEW::LEFT : sl::VIEW::LEFT_UNRECTIFIED;
    const auto right_view = options.rectified ? sl::VIEW::RIGHT : sl::VIEW::RIGHT_UNRECTIFIED;

    auto left_endpoint = create_stream_endpoint(node, options, stream_base + "/left", left_view);
    auto right_endpoint = create_stream_endpoint(node, options, stream_base + "/right", right_view);

    if (!left_endpoint.has_value() && !right_endpoint.has_value()) {
        std::cerr << "[ZED-Iceoryx2] No publishers created for camera " << device_index << std::endl;
        instance->camera_.close();
        return nullptr;
    }

    if (left_endpoint.has_value()) {
        instance->left_ = std::move(left_endpoint.value());
    }
    if (right_endpoint.has_value()) {
        instance->right_ = std::move(right_endpoint.value());
    }

    instance->control_ = create_control_channel(node, stream_base + "/control");
    if (!instance->control_.has_value()) {
        instance->camera_.close();
        return nullptr;
    }

    instance->calibration_channel_ = create_calibration_channel(node, stream_base + "/calibration", instance->serial_number_);
    if (!instance->calibration_channel_.has_value()) {
        instance->camera_.close();
        return nullptr;
    }

    instance->calibration_ = instance->buildCalibrationSnapshot(options);

    const auto camera_info = instance->camera_.getCameraInformation();
    std::cout << "[ZED-Iceoryx2] Opened camera " << device_index
              << " serial=" << camera_info.serial_number
              << " left_service=" << instance->left_.service_name
              << " right_service=" << instance->right_.service_name
              << " control_service=" << instance->control_->service_name
              << " calibration_service=" << instance->calibration_channel_->service_name
              << std::endl;

    return instance;
}

auto CameraInstanceService::deviceIndex() const -> size_t {
    return device_index_;
}

auto CameraInstanceService::serialNumber() const -> uint32_t {
    return serial_number_;
}

auto CameraInstanceService::cameraModel() const -> sl::MODEL {
    return camera_model_;
}

auto CameraInstanceService::instanceName() const -> const std::string& {
    return instance_name_;
}

auto CameraInstanceService::frameCount() const -> uint64_t {
    return frame_count_;
}

auto CameraInstanceService::publishedFrames() const -> uint64_t {
    return published_frames_;
}

auto CameraInstanceService::leftServiceName() const -> const std::string& {
    return left_.service_name;
}

auto CameraInstanceService::rightServiceName() const -> const std::string& {
    return right_.service_name;
}

auto CameraInstanceService::controlServiceName() const -> const std::string& {
    return control_->service_name;
}

auto CameraInstanceService::calibrationServiceName() const -> const std::string& {
    return calibration_channel_->service_name;
}

auto CameraInstanceService::camera() -> sl::Camera& {
    return camera_;
}

auto CameraInstanceService::controlChannel() -> std::optional<ControlChannel>& {
    return control_;
}

auto CameraInstanceService::matchesSelector(const ControlRequest& request) const -> bool {
    const auto selector_kind = static_cast<CameraSelectorKind>(request.selector_kind);
    switch (selector_kind) {
        case CameraSelectorKind::All:
            return true;
        case CameraSelectorKind::Serial:
            return serial_number_ == request.selector_value;
        case CameraSelectorKind::Index:
            return static_cast<uint32_t>(device_index_) == request.selector_value;
    }
    return false;
}

auto CameraInstanceService::makeResponseBase(const ControlRequest& request) const -> ControlResponse {
    ControlResponse response {};
    response.command = request.command;
    response.serial_number = serial_number_;
    response.camera_index = static_cast<uint32_t>(device_index_);
    response.camera_model = static_cast<uint32_t>(camera_model_);
    response.setting = request.setting;
    return response;
}

auto CameraInstanceService::buildCalibrationSnapshot(const Options& options) const -> StereoCalibrationSnapshot {
    const auto requested_resolution =
        options.publish_resolution.has_value()
            ? options.publish_resolution.value()
            : camera_.getCameraInformation().camera_configuration.resolution;
    const auto info = camera_.getCameraInformation(requested_resolution);

    StereoCalibrationSnapshot snapshot {};
    snapshot.serial_number = serial_number_;
    snapshot.camera_index = static_cast<uint32_t>(device_index_);
    snapshot.camera_model = static_cast<uint32_t>(camera_model_);
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

auto CameraInstanceService::writeCalibrationSnapshot(CalibrationChannel& channel) const -> bool {
    if (!channel.writer.has_value()) {
        return false;
    }

    auto entry_result = channel.writer->entry<StereoCalibrationSnapshot>(serial_number_);
    if (!entry_result.has_value()) {
        std::cerr << "[ZED-Iceoryx2] Failed to acquire calibration entry for serial "
                  << serial_number_ << std::endl;
        return false;
    }

    entry_result->update_with_copy(calibration_);
    return true;
}

auto CameraInstanceService::writeCalibrationSnapshots(CalibrationChannel* aggregate_channel) -> void {
    if (calibration_channel_.has_value()) {
        writeCalibrationSnapshot(calibration_channel_.value());
    }
    if (aggregate_channel != nullptr) {
        writeCalibrationSnapshot(*aggregate_channel);
    }
}

auto CameraInstanceService::refreshCalibration(const Options& options, CalibrationChannel* aggregate_channel) -> void {
    calibration_ = buildCalibrationSnapshot(options);
    writeCalibrationSnapshots(aggregate_channel);
}

auto CameraInstanceService::publishImage(StreamEndpoint& endpoint, sl::Mat& image, uint64_t timestamp_ns) -> bool {
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
    header.frame_id = frame_count_;
    header.serial_number = serial_number_;
    header.camera_index = static_cast<uint32_t>(device_index_);
    header.camera_model = static_cast<uint32_t>(camera_model_);
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

    ++published_frames_;
    return true;
}

auto CameraInstanceService::publishCurrentFrames(const Options& options, const sl::RuntimeParameters& runtime_parameters) -> void {
    if (camera_.grab(runtime_parameters) != sl::ERROR_CODE::SUCCESS) {
        return;
    }

    ++frame_count_;
    const auto timestamp_ns = camera_.getTimestamp(sl::TIME_REFERENCE::IMAGE).data_ns;

    if (left_.publisher.has_value()) {
        sl::Mat left_image;
        if (retrieve_image(camera_, left_image, left_.view, options.publish_resolution) == sl::ERROR_CODE::SUCCESS) {
            publishImage(left_, left_image, timestamp_ns);
        }
    }

    if (right_.publisher.has_value()) {
        sl::Mat right_image;
        if (retrieve_image(camera_, right_image, right_.view, options.publish_resolution) == sl::ERROR_CODE::SUCCESS) {
            publishImage(right_, right_image, timestamp_ns);
        }
    }
}
