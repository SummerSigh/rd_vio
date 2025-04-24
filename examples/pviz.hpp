#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <mutex>
#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace pviz{

class Color {
public:
    // from rgba
    Color(const uint8_t r_, const uint8_t g_, const uint8_t b_, const uint8_t a_ = 255) {
        r = r_; g = g_; b = b_; a = a_;
        float_ = {
            r : r_ / 255.0f,
            g : g_ / 255.0f,
            b : b_ / 255.0f,
            a : a_ / 255.0f
        };
    }

    static Color RGBA(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) {
        return Color(r, g, b, a);
    }

    static Color RGBA(const uint8_t r, const uint8_t g, const uint8_t b, const float a) {
        return Color(r, g, b, a * 255);
    }

    static Color RGB(const uint8_t r, const uint8_t g, const uint8_t b) {
        return Color(r, g, b, 255);
    }

    static Color RGBA_HEX(const int32_t hex) {
        return Color(
            (hex >> 24) & 0xFF,
            (hex >> 16) & 0xFF,
            (hex >> 8) & 0xFF,
            hex & 0xFF
        );
    }

    static Color RGB_HEX(const int32_t hex, const uint8_t a = 255) {
        return Color(
            (hex >> 16) & 0xFF,
            (hex >> 8) & 0xFF,
            hex & 0xFF,
            a
        );
    }

    static Color RGB_HEX(const int32_t hex, const float a) {
        return RGB_HEX(hex, (uint8_t)(a * 255));
    }

    uint8_t r, g, b, a;
    struct {
        float r, g, b, a;
    } float_;
}; // class Color

class Settings {
public:
    ///// pangolin window
    int width = 1024;
    int height = 768;
    int focal = 500;
    float near = 0.1;
    float far = 1000.0;
    bool follow = false;
    ///// camera view
    float ex = 0.0;
    float ey = -0.7;
    float ez = -1.8;
    float lx = 0.0;
    float ly = 0.0;
    float lz = 0.0;
    float ux = 0.0;
    float uy = -1.0;
    float uz = 0.0;
    ///// bounds
    float bbottom = 0.0;
    float btop = 1.0;
    float bleft = 0.0;
    float bright = 1.0;
    ///// background
    Color bg = Color::RGB(255, 255, 255);
    Color camera = Color::RGB(0, 0, 255);
    Color kf = Color::RGB(255, 0, 0);
    Color traj = Color::RGB(0, 0, 255);
    Color lmap = Color::RGB(0, 255, 0);
    Color gmap = Color::RGB(100, 100, 100);
    //// trajectory
    float traj_linewidth = 3.0f;
    float axes_size = 0.12f;
    float lmap_size = 3.0f;
    float gmap_size = 3.0f;
    float topic_wait_ms = 10;
    

};  // class Settings

class Viewer {
public:
    // constructor
    Viewer(const std::string& title, const Settings& settings = Settings())
        : title_(title), SE(settings) 
    {}
    // destructor
    ~Viewer() {
        exit();
    }

    // main loop - simplified version without visualization
    void run() {
        std::cout << "Tracking started. Position values will be printed to console." << std::endl;
        std::cout << "Format: [x, y, z]" << std::endl;
        
        // Simple loop that just waits for exit signal
        while(!exit_) {
            // Sleep to avoid high CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Tracking stopped." << std::endl;
    }

    // exit loop
    void exit() {
        exit_ = true;
    }

    // trajectory (Twc) - now just prints position values
    void publish_trajectory(const Eigen::Matrix4d& T) {
        // Extract position (translation) from transformation matrix
        Eigen::Vector3d position = T.block<3, 1>(0, 3);
        
        // Print position values to console
        std::cout << "Position: [" << position(0) << ", " << position(1) << ", " << position(2) << "]" << std::endl;
    }
    void publish_trajectory(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        // Print position values directly from translation vector
        std::cout << "Position: [" << t(0) << ", " << t(1) << ", " << t(2) << "]" << std::endl;
    }

    // Empty placeholder methods for compatibility
    // These methods no longer do anything since we've removed visualization
    
    // keyframe - no-op methods
    void publish_keyframe(const size_t id, const Eigen::Matrix4d& Twc) {
        // No operation needed
    }
    void remove_keyframe(const size_t id) {
        // No operation needed
    }
    void remove_keyframes() {
        // No operation needed
    }

    // global map - no-op methods
    void publish_global_point_cloud(const size_t id, const Eigen::Vector3d& point) {
        // No operation needed
    }
    void remove_global_point_cloud(const size_t id) {
        // No operation needed
    }
    void remove_global_point_clouds() {
        // No operation needed
    }

    // local map - no-op methods
    void publish_local_point_cloud(const std::vector<Eigen::Vector3d>& points, const bool copy = false) {
        // No operation needed
    }
    void remove_local_point_cloud() {
        // No operation needed
    }

    // extra topic - no-op method
    void publish_topic(const std::string& topic, const cv::Mat& img) {
        // No operation needed
    }

private:
    // No drawing functions needed anymore as we're just printing position values

    ///// simplified member variables
    const Settings SE;
    const std::string title_;   // title
    std::atomic<bool> exit_ = false;    // exit signal

}; // class Viewer

} // namespace pviz
