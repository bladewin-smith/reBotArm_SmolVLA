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
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
    bool align_depth_to_color = false;
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
        else if(key == "--align-depth-to-color") {
            args.align_depth_to_color = true;
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

void ensure_file_exists(const std::string& path, const std::string& label) {
    std::ifstream file(path);
    if(!file.good()) {
        throw std::runtime_error(label + " does not exist or is not readable: " + path);
    }
}

std::shared_ptr<ob::Filter> create_enhanced_depth_filter(const Args& args) {
    if(!args.enhanced_depth_filter) {
        return nullptr;
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
        filter = ob::FilterFactory::createPrivateFilter(
            args.enhanced_depth_filter_name.c_str(),
            args.enhanced_depth_model.c_str()
        );
        if(filter == nullptr) {
            throw std::runtime_error("OrbbecSDK returned a null EnhancedDepthFilter.");
        }
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

void write_packet(const std::vector<uint8_t>& color_rgb, const std::vector<uint16_t>& depth_mm, int width, int height) {
    PacketHeader header{};
    std::memcpy(header.magic, "OBLR", 4);
    header.timestamp_ns = now_ns();
    header.width = static_cast<uint32_t>(width);
    header.height = static_cast<uint32_t>(height);
    header.color_size = static_cast<uint32_t>(color_rgb.size());
    header.depth_size = static_cast<uint32_t>(depth_mm.size() * sizeof(uint16_t));

    std::cout.write(reinterpret_cast<const char*>(&header), sizeof(header));
    if(!color_rgb.empty()) {
        std::cout.write(reinterpret_cast<const char*>(color_rgb.data()), static_cast<std::streamsize>(color_rgb.size()));
    }
    if(!depth_mm.empty()) {
        std::cout.write(
            reinterpret_cast<const char*>(depth_mm.data()),
            static_cast<std::streamsize>(depth_mm.size() * sizeof(uint16_t))
        );
    }
    std::cout.flush();
}

std::shared_ptr<ob::Device> get_device_by_serial(ob::Context& context, const std::string& serial) {
    auto devices = context.queryDeviceList();
    if(devices == nullptr || devices->getCount() == 0) {
        throw std::runtime_error("No Orbbec devices found");
    }

    try {
        return devices->getDeviceBySN(serial.c_str());
    }
    catch(const std::exception&) {
        std::ostringstream available;
        for(uint32_t index = 0; index < devices->getCount(); ++index) {
            auto device = devices->getDevice(index);
            auto info = device->getDeviceInfo();
            if(info == nullptr) {
                continue;
            }
            if(index > 0) {
                available << ", ";
            }
            available << info->serialNumber();
        }
        throw std::runtime_error(
            "Could not find Orbbec device with serial " + serial + ". Available serials: " + available.str()
        );
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        ob::Context context;
        if(!args.serial.empty()) {
            context.setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList>, std::shared_ptr<ob::DeviceList>) {});
        }

        std::shared_ptr<ob::Pipeline> pipeline;
        if(args.serial.empty()) {
            pipeline = std::make_shared<ob::Pipeline>();
        }
        else {
            pipeline = std::make_shared<ob::Pipeline>(get_device_by_serial(context, args.serial));
        }

        auto config = std::make_shared<ob::Config>();

        auto color_profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        auto color_profile = choose_profile(color_profiles, args.width, args.height, OB_FORMAT_RGB, args.fps);
        config->enableStream(color_profile);

        auto depth_profiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        auto depth_profile = choose_profile(depth_profiles, args.width, args.height, OB_FORMAT_Y16, args.fps);
        config->enableStream(depth_profile);

        if(args.align_depth_to_color) {
            config->setAlignMode(ALIGN_D2C_HW_MODE);
        }

        auto enhanced_depth_filter = create_enhanced_depth_filter(args);

        pipeline->start(config);

        while(true) {
            auto frameset = pipeline->waitForFrames(1000);
            if(frameset == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if(frameset->colorFrame() == nullptr || frameset->depthFrame() == nullptr) {
                continue;
            }

            frameset = apply_enhanced_depth_filter(frameset, enhanced_depth_filter);

            auto color_frame = frameset->colorFrame();
            auto depth_frame = frameset->depthFrame();
            if(color_frame == nullptr || depth_frame == nullptr) {
                continue;
            }

            const int width = static_cast<int>(color_frame->width());
            const int height = static_cast<int>(color_frame->height());
            const int depth_width = static_cast<int>(depth_frame->width());
            const int depth_height = static_cast<int>(depth_frame->height());
            if(depth_width != width || depth_height != height) {
                throw std::runtime_error(
                    "Depth frame size " + std::to_string(depth_width) + "x" + std::to_string(depth_height)
                    + " does not match color frame size " + std::to_string(width) + "x" + std::to_string(height)
                    + ". Enable --align-depth-to-color or request matching RGB/depth stream profiles."
                );
            }

            const auto color_bytes = static_cast<size_t>(width * height * 3);
            std::vector<uint8_t> color_rgb(color_bytes);
            std::memcpy(color_rgb.data(), color_frame->data(), color_bytes);

            const auto depth_pixels = static_cast<size_t>(width * height);
            std::vector<uint16_t> depth_mm(depth_pixels);
            std::memcpy(depth_mm.data(), depth_frame->data(), depth_pixels * sizeof(uint16_t));

            write_packet(color_rgb, depth_mm, width, height);
        }
    }
    catch(const std::exception& exc) {
        std::cerr << "orbbec_rgbd_bridge failed: " << exc.what() << std::endl;
        return 1;
    }
    return 0;
}
