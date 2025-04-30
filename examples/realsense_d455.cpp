// Copyright (c) 2024 RDVIO
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <rdvio/rdvio.h>
#include <rdvio/extra/yaml_config.h>
#include <rdvio/extra/drawing.h>

std::atomic<bool> should_exit(false);
std::mutex data_mutex;
std::vector<std::pair<double, cv::Mat>> image_buffer;
std::vector<rdvio::ImuData> imu_buffer;

// Function to handle IMU data
void imu_callback(rs2::frame frame) {
    auto motion = frame.as<rs2::motion_frame>();
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
        double timestamp = motion.get_timestamp() / 1000.0; // Convert ms to s
        rs2_vector data = motion.get_motion_data();
        std::lock_guard<std::mutex> lock(data_mutex);
        // Find the corresponding accelerometer data (assuming they arrive close together)
        for (auto it = imu_buffer.rbegin(); it != imu_buffer.rend(); ++it) {
            if (std::abs(it->timestamp - timestamp) < 0.001) { // Match within 1ms
                it->gyro = Eigen::Vector3d(data.x, data.y, data.z);
                break;
            }
        }
    }
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
        double timestamp = motion.get_timestamp() / 1000.0; // Convert ms to s
        rs2_vector data = motion.get_motion_data();
        std::lock_guard<std::mutex> lock(data_mutex);
        imu_buffer.push_back({timestamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(data.x, data.y, data.z)});
    }
}

// Function to handle Image data
void frame_callback(rs2::frame frame) {
    auto frameset = frame.as<rs2::frameset>();
    if (frameset) {
        rs2::video_frame image_frame = frameset.get_color_frame(); // Or get_infrared_frame(1)
        if (image_frame) {
            double timestamp = image_frame.get_timestamp() / 1000.0; // Convert ms to s
            int width = image_frame.get_width();
            int height = image_frame.get_height();

            cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)image_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

            std::lock_guard<std::mutex> lock(data_mutex);
            image_buffer.emplace_back(timestamp, gray_image.clone());
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path-to-device-config> <path-to-setting-config>" << std::endl;
        return 1;
    }

    std::string device_config_path = argv[1];
    std::string setting_config_path = argv[2];

    // Load configuration
    std::shared_ptr<rdvio::Config> config;
    try {
        config = std::make_shared<rdvio::extra::YamlConfig>(setting_config_path, device_config_path);
    } catch (const std::exception &e) {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
        return 1;
    }

    // Initialize RDVIO system
    rdvio::System system(config);
    rdvio::extra::Viewer viewer(config);

    // Initialize RealSense
    rs2::pipeline pipe;
    rs2::config rs_cfg;

    // Enable streams - Adjust resolution and framerate as needed for D455
    // Check D455 specs for supported modes
    rs_cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); // D455 typically supports 250Hz or 400Hz
    rs_cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);  // D455 typically supports 200Hz or 400Hz
    rs_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30); // Example: 640x480 @ 30fps
    // Or use infrared: rs_cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

    // Start pipeline with callbacks
    rs2::pipeline_profile profile = pipe.start(rs_cfg, [&](rs2::frame frame) {
        if (auto motion = frame.as<rs2::motion_frame>()) {
            imu_callback(frame);
        } else if (auto fs = frame.as<rs2::frameset>()) {
            frame_callback(frame);
        }
    });

    std::cout << "RealSense pipeline started." << std::endl;

    // Main loop to process data
    std::thread viewer_thread([&]() {
        while (!should_exit) {
            viewer.spin_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    while (!should_exit) {
        std::vector<std::pair<double, cv::Mat>> local_image_buffer;
        std::vector<rdvio::ImuData> local_imu_buffer;

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_image_buffer.swap(image_buffer);
            local_imu_buffer.swap(imu_buffer);
        }

        // Feed IMU data
        for (const auto& imu_data : local_imu_buffer) {
            system.feed_imu(imu_data);
        }

        // Feed Image data
        for (const auto& img_pair : local_image_buffer) {
            system.feed_image(img_pair.first, img_pair.second);
            viewer.update(system, img_pair.second);
        }

        // Check for viewer exit request
        if (viewer.should_quit()) {
            should_exit = true;
        }

        // Avoid busy-waiting
        if (local_image_buffer.empty() && local_imu_buffer.empty()) {
             std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::cout << "Stopping RealSense pipeline..." << std::endl;
    pipe.stop();
    viewer_thread.join();
    std::cout << "Exiting." << std::endl;

    return 0;
}