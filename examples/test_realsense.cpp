#include <chrono>
#include <thread>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <rdvio/rdvio.hpp>
#include "pviz.hpp"

int main(int argc, char** argv) {
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file>" << std::endl;
        std::cerr << "  - Generate calibration file with: ./realsense_d455 <output_yaml>" << std::endl;
        return 1;
    }

    const std::string calib_file = argv[1];
    const std::string config_file = argv[2];

    // Create RealSense context
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No RealSense devices found!" << std::endl;
        return 1;
    }

    std::cout << "Found " << devices.size() << " RealSense device(s)" << std::endl;
    rs2::device device = devices[0];
    std::cout << "Using device: " << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << "Serial Number: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

    // Configure the pipeline
    rs2::pipeline pipe;
    rs2::config cfg;

    // Print available sensors on the device
    std::cout << "\nAvailable sensors on this device:" << std::endl;
    for (auto& sensor : device.query_sensors()) {
        std::cout << "  - " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    
    // Try flexible stream configuration (works for more devices)
    try {
        // Enable infrared stream for SLAM (not depth or color)
        std::cout << "  - Enabling Left IR Stream..." << std::endl;
        cfg.enable_stream(RS2_STREAM_INFRARED, 1);
        
        // Enable gyro and accelerometer
        std::cout << "  - Enabling Accelerometer..." << std::endl;
        cfg.enable_stream(RS2_STREAM_ACCEL);
        
        std::cout << "  - Enabling Gyroscope..." << std::endl;
        cfg.enable_stream(RS2_STREAM_GYRO);
    } catch (const rs2::error& e) {
        std::cerr << "\nError configuring streams: " << e.what() << std::endl;
        std::cerr << "Trying fallback configuration..." << std::endl;
        
        // Clear configuration
        cfg = rs2::config();
        
        // Use specific settings
        std::cout << "  - Enabling Left IR Stream (640x480 @ 30fps)..." << std::endl;
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
        
        std::cout << "  - Enabling Accelerometer (200Hz)..." << std::endl;
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
        
        std::cout << "  - Enabling Gyroscope (200Hz)..." << std::endl;
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    }

    // Start the pipeline
    std::cout << "Starting RealSense pipeline..." << std::endl;
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Initialize the VIO system
    std::cout << "Initializing VIO with calibration from: " << calib_file << std::endl;
    auto vio = rdvio::Odometry(calib_file, config_file);
    
    // Initialize visualization
    auto viewer_setting = pviz::Settings();
    viewer_setting.follow = true;
    auto viewer = pviz::Viewer("realsense", viewer_setting);
    std::thread viewer_thread(&pviz::Viewer::run, &viewer);

    double timestamp_offset = 0;
    bool first_frame = true;
    
    // Tracking stats
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        while (true) {
            // Wait for frames
            rs2::frameset frames = pipe.wait_for_frames();

            // Process IMU data
            if (auto accel_frame = frames.first_or_default(RS2_STREAM_ACCEL)) {
                rs2_vector accel_data = accel_frame.as<rs2::motion_frame>().get_motion_data();
                rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
                
                if (gyro_frame) {
                    rs2_vector gyro_data = gyro_frame.as<rs2::motion_frame>().get_motion_data();
                    
                    // Get timestamps and convert to seconds
                    double accel_timestamp = accel_frame.get_timestamp() / 1000.0;
                    double gyro_timestamp = gyro_frame.get_timestamp() / 1000.0;
                    
                    // Use average timestamp for simplicity
                    double timestamp = (accel_timestamp + gyro_timestamp) / 2.0;
                    
                    if (first_frame) {
                        timestamp_offset = timestamp;
                        first_frame = false;
                    }
                    
                    double time_sec = timestamp - timestamp_offset;
                    
                    // Add motion data to VIO
                    Eigen::Vector3d accel(accel_data.x, accel_data.y, accel_data.z);
                    Eigen::Vector3d gyro(gyro_data.x, gyro_data.y, gyro_data.z);
                    
                    vio.addMotion(time_sec, accel, gyro);
                }
            }

            // Process infrared frame
            if (auto ir_frame = frames.get_infrared_frame(1)) { // Get the left infrared frame
                double timestamp = ir_frame.get_timestamp() / 1000.0;
                double time_sec = timestamp - timestamp_offset;
                
                // Convert RealSense frame to OpenCV Mat - infrared is already grayscale (Y8 format)
                const int w = ir_frame.get_width();
                const int h = ir_frame.get_height();
                cv::Mat ir(cv::Size(w, h), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);
                
                // Add frame to VIO - no need to convert to grayscale as it's already in grayscale format
                auto tracking_start = std::chrono::high_resolution_clock::now();
                vio.addFrame(time_sec, ir);
                auto tracking_end = std::chrono::high_resolution_clock::now();
                auto tracking_duration = std::chrono::duration_cast<std::chrono::milliseconds>(tracking_end - tracking_start).count();
                
                frame_count++;
                
                // Display the input image - convert to BGR for visualization
                cv::Mat display;
                cv::cvtColor(ir, display, cv::COLOR_GRAY2BGR);
                viewer.publish_topic("input", display);
                
                // Calculate and show FPS
                auto current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                if (elapsed >= 1) {
                    double fps = static_cast<double>(frame_count) / elapsed;
                    std::cout << "FPS: " << std::fixed << std::setprecision(1) << fps 
                              << " | Tracking time: " << tracking_duration << "ms" << std::endl;
                    frame_count = 0;
                    start_time = current_time;
                }
            }

            // Display point cloud if VIO is initialized
            if (vio.state() == 1) {
                viewer.publish_local_point_cloud(vio.local_map());
                
                // Display current pose
                Eigen::Matrix4d T = vio.transform_world_cam();
                std::cout << "Position: [" 
                          << std::fixed << std::setprecision(3)
                          << T(0,3) << ", " 
                          << T(1,3) << ", " 
                          << T(2,3) << "]" << std::endl;
            }
            
            // Check for exit
            if (cv::waitKey(1) == 27) { // ESC key
                break;
            }
        }
    }
    catch (const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Clean up
    std::cout << "Shutting down..." << std::endl;
    viewer.exit();
    viewer_thread.join();
    pipe.stop();

    return 0;
}