// Copyright 2026 The HuggingFace Inc. team. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Minimal OrbbecSDK v2 C++ bridge for LeRobot.
//
// Stdout protocol, repeated forever:
//   struct Header {
//     char     magic[4] = "OBLR";
//     uint64_t timestamp_ns;
//     uint32_t width;
//     uint32_t height;
//     uint32_t color_size;  // RGB uint8 bytes, H * W * 3, or 0
//     uint32_t depth_size;  // uint16 bytes, H * W * 2, or 0
//   };
//   uint8_t color[color_size];
//   uint8_t depth[depth_size];
//
// The Orbbec LingBot EnhancedDepthFilter is applied at the OrbbecSDK FrameSet
// level, before copying depth bytes into the LeRobot packet. This keeps the
// recorded uint16 millimeter depth map aligned with the RGB frame and ensures
// SDK license/model failures stop collection instead of silently recording raw
// depth as enhanced depth.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <unistd.h>
#endif

#include "libobsensor/ObSensor.hpp"

namespace {

#pragma pack(push, 1)
struct PacketHeader {
    char magic[4];
    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint32_t color_size;
    uint32_t depth_size;
};
#pragma pack(pop)

struct Args {
    int width = 640;
    int height = 480;
    int fps = 30;
    std::string serial;
    bool list_devices = false;
    bool enable_depth = true;
    bool align_depth_to_color = false;
    std::string align_depth_to_color_mode = "sw";
    bool enhanced_depth_filter = false;
    std::string enhanced_depth_filter_name = "EnhancedDepthFilter";
    std::string enhanced_depth_model;
    std::string enhanced_depth_confidence_key = "confidence_threshold";
    int enhanced_depth_confidence_threshold = 51;
};

uint64_t now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

Args parse_args(int argc, char** argv) {
    Args args;
    for(int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        auto require_value = [&](const char* name) -> std::string {
            if(i + 1 >= argc) {
                throw std::runtime_error(std::string("Missing value for ") + name);
            }
            return std::string(argv[++i]);
        };

        if(key == "--width") {
            args.width = std::stoi(require_value("--width"));
        }
        else if(key == "--height") {
            args.height = std::stoi(require_value("--height"));
        }
        else if(key == "--fps") {
            args.fps = std::stoi(require_value("--fps"));
        }
        else if(key == "--serial") {
            args.serial = require_value("--serial");
        }
        else if(key == "--list-devices") {
            args.list_devices = true;
        }
        else if(key == "--disable-depth" || key == "--color-only") {
            args.enable_depth = false;
        }
        else if(key == "--align-depth-to-color") {
            args.align_depth_to_color = true;
        }
        else if(key == "--align-depth-to-color-mode") {
            args.align_depth_to_color_mode = require_value("--align-depth-to-color-mode");
        }
        else if(key == "--enhanced-depth-filter" || key == "--lingbo-filter") {
            args.enhanced_depth_filter = true;
        }
        else if(key == "--enhanced-depth-filter-name") {
            args.enhanced_depth_filter_name = require_value("--enhanced-depth-filter-name");
        }
        else if(key == "--enhanced-depth-model" || key == "--lingbo-model") {
            args.enhanced_depth_model = require_value(key.c_str());
        }
        else if(key == "--enhanced-depth-confidence-key") {
            args.enhanced_depth_confidence_key = require_value("--enhanced-depth-confidence-key");
        }
        else if(key == "--enhanced-depth-confidence-threshold") {
            args.enhanced_depth_confidence_threshold =
                std::stoi(require_value("--enhanced-depth-confidence-threshold"));
        }
        else {
            throw std::runtime_error("Unknown argument: " + key);
        }
    }
    return args;
}

std::string value_or_error(const std::function<std::string()>& fn) {
    try {
        return fn();
    }
    catch(const std::exception& exc) {
        return std::string("<error: ") + exc.what() + ">";
    }
}

std::shared_ptr<ob::VideoStreamProfile> choose_profile(
    const std::shared_ptr<ob::StreamProfileList>& profiles,
    int width,
    int height,
    OBFormat format,
    int fps
) {
    try {
        return profiles->getVideoStreamProfile(width, height, format, fps);
    }
    catch(...) {
        return profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, format, fps);
    }
}

std::shared_ptr<ob::VideoStreamProfile> choose_color_profile(
    const std::shared_ptr<ob::StreamProfileList>& profiles,
    int width,
    int height,
    int fps
) {
    try {
        return profiles->getVideoStreamProfile(width, height, OB_FORMAT_RGB, fps);
    }
    catch(...) {
    }
    try {
        return profiles->getVideoStreamProfile(width, height, OB_FORMAT_ANY, fps);
    }
    catch(...) {
    }
    return profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, fps);
}

void ensure_file_exists(const std::string& path, const std::string& label) {
    std::ifstream file(path);
    if(!file.good()) {
        throw std::runtime_error(label + " does not exist or is not readable: " + path);
    }
}

OBConvertFormat convert_format_for_rgb(OBFormat format) {
    switch(format) {
        case OB_FORMAT_YUYV:
            return FORMAT_YUYV_TO_RGB;
        case OB_FORMAT_UYVY:
            return FORMAT_UYVY_TO_RGB;
        case OB_FORMAT_NV12:
            return FORMAT_NV12_TO_RGB;
        case OB_FORMAT_NV21:
            return FORMAT_NV21_TO_RGB;
        case OB_FORMAT_I420:
            return FORMAT_I420_TO_RGB;
        case OB_FORMAT_MJPG:
            return FORMAT_MJPG_TO_RGB;
        default:
            throw std::runtime_error("Unsupported Orbbec color format for RGB conversion: " + std::to_string(format));
    }
}

std::shared_ptr<ob::VideoFrame> ensure_rgb_color_frame(const std::shared_ptr<ob::VideoFrame>& color_frame) {
    if(color_frame == nullptr) {
        return nullptr;
    }
    if(color_frame->format() == OB_FORMAT_RGB) {
        return color_frame;
    }

    ob::FormatConvertFilter converter;
    converter.setFormatConvertType(convert_format_for_rgb(color_frame->format()));
    auto converted_frame = converter.process(color_frame);
    if(converted_frame == nullptr) {
        throw std::runtime_error("Orbbec FormatConvertFilter returned no RGB frame.");
    }
    auto rgb_frame = converted_frame->as<ob::VideoFrame>();
    if(rgb_frame == nullptr || rgb_frame->format() != OB_FORMAT_RGB) {
        throw std::runtime_error("Orbbec FormatConvertFilter output is not an RGB video frame.");
    }
    return rgb_frame;
}

std::string video_profile_summary(const std::shared_ptr<ob::VideoStreamProfile>& profile) {
    if(profile == nullptr) {
        return "<null>";
    }
    return std::to_string(profile->getWidth()) + "x" + std::to_string(profile->getHeight()) + "@"
           + std::to_string(profile->getFps()) + " format=" + std::to_string(profile->getFormat());
}

std::shared_ptr<ob::Filter> create_enhanced_depth_filter(
    const Args& args,
    const std::shared_ptr<ob::Device>& device
) {
    if(!args.enhanced_depth_filter) {
        return nullptr;
    }
    if(!args.enable_depth) {
        throw std::runtime_error("EnhancedDepthFilter requires depth to be enabled.");
    }
    if(device == nullptr) {
        throw std::runtime_error("EnhancedDepthFilter requires a valid Orbbec device.");
    }
    if(args.enhanced_depth_model.empty()) {
        throw std::runtime_error("--enhanced-depth-model is required when --enhanced-depth-filter is enabled.");
    }
    if(args.enhanced_depth_filter_name.empty()) {
        throw std::runtime_error("--enhanced-depth-filter-name cannot be empty.");
    }
    if(args.enhanced_depth_confidence_key.empty()) {
        throw std::runtime_error("--enhanced-depth-confidence-key cannot be empty.");
    }
    if(args.enhanced_depth_confidence_threshold < 0 || args.enhanced_depth_confidence_threshold > 255) {
        throw std::runtime_error("--enhanced-depth-confidence-threshold must be in [0, 255].");
    }

    ensure_file_exists(args.enhanced_depth_model, "EnhancedDepthFilter model");

    std::shared_ptr<ob::Filter> filter;
    try {
        if(args.enhanced_depth_filter_name == "EnhancedDepthFilter") {
            filter = std::make_shared<ob::EnhancedDepthFilter>(device, args.enhanced_depth_model);
        }
        else {
            filter = ob::FilterFactory::createPrivateFilter(
                args.enhanced_depth_filter_name.c_str(),
                args.enhanced_depth_model.c_str()
            );
        }
        if(filter == nullptr) {
            throw std::runtime_error("OrbbecSDK returned a null EnhancedDepthFilter.");
        }
        filter->setConfigValue("width", static_cast<double>(args.width));
        filter->setConfigValue("height", static_cast<double>(args.height));
        filter->setConfigValue(
            args.enhanced_depth_confidence_key.c_str(),
            static_cast<double>(args.enhanced_depth_confidence_threshold)
        );
        filter->enable(true);
    }
    catch(const std::exception& exc) {
        throw std::runtime_error(
            "Failed to create EnhancedDepthFilter. Check that Jetson Linux ARM64, CUDA 12, "
            "TensorRT 10, the EnhancedDepthFilter extension library, model.sm4, OrbbecSDK, "
            "and the device LingBot-Depth license all come from a compatible Orbbec release. "
            "Original error: "
            + std::string(exc.what())
        );
    }

    std::cerr << args.enhanced_depth_filter_name << " enabled with model: " << args.enhanced_depth_model
              << ", " << args.enhanced_depth_confidence_key << ": "
              << args.enhanced_depth_confidence_threshold << std::endl;
    return filter;
}

std::shared_ptr<ob::FrameSet> apply_enhanced_depth_filter(
    const std::shared_ptr<ob::FrameSet>& frameset,
    const std::shared_ptr<ob::Filter>& filter
) {
    if(filter == nullptr) {
        return frameset;
    }
    if(frameset == nullptr || frameset->colorFrame() == nullptr || frameset->depthFrame() == nullptr) {
        throw std::runtime_error("EnhancedDepthFilter requires a frameset containing both color and depth frames.");
    }

    std::shared_ptr<ob::Frame> filtered_frame;
    try {
        filtered_frame = filter->process(frameset);
    }
    catch(const std::exception& exc) {
        throw std::runtime_error(
            "EnhancedDepthFilter processing failed. The frame must be an aligned color+depth frameset. "
            "Original error: "
            + std::string(exc.what())
        );
    }

    if(filtered_frame == nullptr) {
        throw std::runtime_error("EnhancedDepthFilter returned no frame.");
    }

    auto filtered_frameset = filtered_frame->as<ob::FrameSet>();
    if(filtered_frameset == nullptr) {
        throw std::runtime_error("EnhancedDepthFilter output is not a FrameSet.");
    }
    if(filtered_frameset->colorFrame() == nullptr || filtered_frameset->depthFrame() == nullptr) {
        throw std::runtime_error("EnhancedDepthFilter output is missing color or depth frame.");
    }

    return filtered_frameset;
}

void write_all(int fd, const void* data, size_t size) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    size_t written = 0;
    while(written < size) {
#ifdef _WIN32
        std::cout.write(
            reinterpret_cast<const char*>(bytes + written),
            static_cast<std::streamsize>(size - written)
        );
        if(!std::cout.good()) {
            throw std::runtime_error("Failed to write Orbbec frame packet to stdout.");
        }
        std::cout.flush();
        return;
#else
        const ssize_t rc = ::write(fd, bytes + written, size - written);
        if(rc <= 0) {
            throw std::runtime_error("Failed to write Orbbec frame packet to stdout.");
        }
        written += static_cast<size_t>(rc);
#endif
    }
}

void write_packet(
    int protocol_fd,
    const std::vector<uint8_t>& color_rgb,
    const std::vector<uint16_t>& depth_mm,
    int width,
    int height
) {
    PacketHeader header{};
    std::memcpy(header.magic, "OBLR", 4);
    header.timestamp_ns = now_ns();
    header.width = static_cast<uint32_t>(width);
    header.height = static_cast<uint32_t>(height);
    header.color_size = static_cast<uint32_t>(color_rgb.size());
    header.depth_size = static_cast<uint32_t>(depth_mm.size() * sizeof(uint16_t));

    write_all(protocol_fd, &header, sizeof(header));
    if(!color_rgb.empty()) {
        write_all(protocol_fd, color_rgb.data(), color_rgb.size());
    }
    if(!depth_mm.empty()) {
        write_all(protocol_fd, depth_mm.data(), depth_mm.size() * sizeof(uint16_t));
    }
}

std::string device_list_summary(const std::shared_ptr<ob::DeviceList>& devices) {
    if(devices == nullptr) {
        return "No Orbbec device list returned by SDK.";
    }

    std::ostringstream out;
    const uint32_t count = devices->getCount();
    out << "Orbbec devices found: " << count;
    for(uint32_t index = 0; index < count; ++index) {
        out << "\n[" << index << "]"
            << " name=" << value_or_error([&]() { return std::string(devices->getName(index)); })
            << " serial=" << value_or_error([&]() { return std::string(devices->getSerialNumber(index)); })
            << " uid=" << value_or_error([&]() { return std::string(devices->getUid(index)); })
            << " connection=" << value_or_error([&]() { return std::string(devices->getConnectionType(index)); })
            << " vid=" << value_or_error([&]() { return std::to_string(devices->getVid(index)); })
            << " pid=" << value_or_error([&]() { return std::to_string(devices->getPid(index)); });
    }
    return out.str();
}

std::shared_ptr<ob::Device> get_device_by_serial(ob::Context& context, const std::string& serial) {
    auto devices = context.queryDeviceList();
    if(devices == nullptr || devices->getCount() == 0) {
        throw std::runtime_error("No Orbbec devices found");
    }

    std::string direct_error;
    try {
        return devices->getDeviceBySN(serial.c_str());
    }
    catch(const std::exception& exc) {
        direct_error = exc.what();
    }

    std::ostringstream available;
    std::string errors;
    for(uint32_t index = 0; index < devices->getCount(); ++index) {
        try {
            if(index > 0) {
                available << ", ";
            }
            const std::string candidate_serial = devices->getSerialNumber(index);
            available << candidate_serial;
            if(candidate_serial == serial) {
                return devices->getDevice(index);
            }
        }
        catch(const std::exception& exc) {
            if(!errors.empty()) {
                errors += "; ";
            }
            errors += "index " + std::to_string(index) + ": " + exc.what();
        }
    }

    std::string message = "Could not find Orbbec device with serial " + serial
                          + ". Available serials: " + available.str();
    if(!direct_error.empty()) {
        message += ". getDeviceBySN error: " + direct_error;
    }
    if(!errors.empty()) {
        message += ". Device list errors: " + errors;
    }
    throw std::runtime_error(message);
}

}  // namespace

int main(int argc, char** argv) {
    try {
#ifdef _WIN32
        const int protocol_fd = 1;
#else
        const int protocol_fd = ::dup(STDOUT_FILENO);
        if(protocol_fd < 0) {
            throw std::runtime_error("Failed to duplicate stdout for Orbbec frame packets.");
        }
        if(::dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            throw std::runtime_error("Failed to redirect stdout to stderr for OrbbecSDK logs.");
        }
#endif

        const Args args = parse_args(argc, argv);

        ob::Context context;
        if(args.list_devices) {
            std::cerr << device_list_summary(context.queryDeviceList()) << std::endl;
            return 0;
        }

        if(!args.serial.empty()) {
            context.setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList>, std::shared_ptr<ob::DeviceList>) {});
        }

        std::shared_ptr<ob::Pipeline> pipeline;
        std::shared_ptr<ob::Device> device;
        if(args.serial.empty()) {
            pipeline = std::make_shared<ob::Pipeline>();
            device = pipeline->getDevice();
        }
        else {
            device = get_device_by_serial(context, args.serial);
            pipeline = std::make_shared<ob::Pipeline>(device);
        }

        auto config = std::make_shared<ob::Config>();

        auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        auto color_profile = choose_color_profile(color_profiles, args.width, args.height, args.fps);
        config->enableStream(color_profile);
        std::cerr << "Selected color profile: " << video_profile_summary(color_profile) << std::endl;

        if(args.enable_depth) {
            auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            auto depth_profile = choose_profile(depth_profiles, args.width, args.height, OB_FORMAT_Y16, args.fps);
            config->enableStream(depth_profile);
            std::cerr << "Selected depth profile: " << video_profile_summary(depth_profile) << std::endl;
        }

        if(args.align_depth_to_color) {
            if(!args.enable_depth) {
                throw std::runtime_error("--align-depth-to-color requires depth to be enabled.");
            }
            if(args.align_depth_to_color_mode == "hw" || args.align_depth_to_color_mode == "hardware") {
                config->setAlignMode(ALIGN_D2C_HW_MODE);
            }
            else if(args.align_depth_to_color_mode == "sw" || args.align_depth_to_color_mode == "software") {
                config->setAlignMode(ALIGN_D2C_SW_MODE);
            }
            else {
                throw std::runtime_error(
                    "--align-depth-to-color-mode must be 'sw', 'software', 'hw', or 'hardware'."
                );
            }
        }

        auto enhanced_depth_filter = create_enhanced_depth_filter(args, device);

        pipeline->start(config);

        while(true) {
            auto frameset = pipeline->waitForFrames(1000);
            if(frameset == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if(frameset->colorFrame() == nullptr) {
                continue;
            }
            if(args.enable_depth && frameset->depthFrame() == nullptr) {
                continue;
            }

            frameset = apply_enhanced_depth_filter(frameset, enhanced_depth_filter);

            auto color_frame = ensure_rgb_color_frame(frameset->colorFrame());
            auto depth_frame = frameset->depthFrame();
            if(color_frame == nullptr) {
                continue;
            }

            const int width = static_cast<int>(color_frame->width());
            const int height = static_cast<int>(color_frame->height());

            const auto color_bytes = static_cast<size_t>(width * height * 3);
            std::vector<uint8_t> color_rgb(color_bytes);
            std::memcpy(color_rgb.data(), color_frame->data(), color_bytes);

            std::vector<uint16_t> depth_mm;
            if(args.enable_depth) {
                if(depth_frame == nullptr) {
                    continue;
                }
                const int depth_width = static_cast<int>(depth_frame->width());
                const int depth_height = static_cast<int>(depth_frame->height());
                if(depth_width != width || depth_height != height) {
                    throw std::runtime_error(
                        "Depth frame size " + std::to_string(depth_width) + "x" + std::to_string(depth_height)
                        + " does not match color frame size " + std::to_string(width) + "x" + std::to_string(height)
                        + ". Enable --align-depth-to-color or request matching RGB/depth stream profiles."
                    );
                }
                const auto depth_pixels = static_cast<size_t>(width * height);
                depth_mm.resize(depth_pixels);
                std::memcpy(depth_mm.data(), depth_frame->data(), depth_pixels * sizeof(uint16_t));
            }

            write_packet(protocol_fd, color_rgb, depth_mm, width, height);
        }
    }
    catch(const std::exception& exc) {
        std::cerr << "orbbec_rgbd_bridge failed: " << exc.what() << std::endl;
        return 1;
    }
    return 0;
}
