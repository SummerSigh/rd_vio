#include <chrono>
#include <thread>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <rdvio/rdvio.hpp>
#include "pviz.hpp"

int main(int argc, char** argv) {
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file>" << std::endl;
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

    // Configure the pipeline
    rs2::pipeline pipe;
    rs2::config cfg;

    // Enable color and depth streams
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    
    // Enable gyro and accelerometer
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    // Start the pipeline
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Initialize the VIO system
    auto vio = rdvio::Odometry(calib_file, config_file);
    
    // Initialize visualization
    auto viewer_setting = pviz::Settings();
    viewer_setting.follow = true;
    auto viewer = pviz::Viewer("realsense", viewer_setting);
    std::thread viewer_thread(&pviz::Viewer::run, &viewer);

    double timestamp_offset = 0;
    bool first_frame = true;

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
                    
                    // Apply coordinate system transforms if needed
                    // RealSense: x right, y down, z forward
                    // Convert to standard coordinate system if needed by the VIO
                    Eigen::Vector3d accel(accel_data.x, accel_data.y, accel_data.z);
                    Eigen::Vector3d gyro(gyro_data.x, gyro_data.y, gyro_data.z);
                    
                    // Add motion data to VIO
                    vio.addMotion(time_sec, accel, gyro);
                }
            }

            // Process color frame
            if (auto color_frame = frames.get_color_frame()) {
                double timestamp = color_frame.get_timestamp() / 1000.0;
                double time_sec = timestamp - timestamp_offset;
                
                // Convert RealSense frame to OpenCV Mat
                const int w = color_frame.get_width();
                const int h = color_frame.get_height();
                cv::Mat color(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                
                // Convert to grayscale for VIO processing
                cv::Mat gray;
                cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
                
                // Add frame to VIO
                vio.addFrame(time_sec, gray);
                
                // Display the input image
                viewer.publish_topic("input", color);
            }

            // Display point cloud if VIO is initialized
            if (vio.state() == 1) {
                viewer.publish_local_point_cloud(vio.local_map());
                
                // Display current pose
                Eigen::Matrix4d T = vio.transform_world_cam();
                std::cout << "Position: [" 
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
    viewer.exit();
    viewer_thread.join();
    pipe.stop();

    return 0;
}