#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rdvio/rdvio.hpp>
#include "pviz.hpp"

// Function declarations
void print_intrinsics(const rs2_intrinsics& intrinsics, std::ostream& out);
void print_motion_intrinsics(const rs2_motion_device_intrinsic& intrinsics, std::ostream& out);
void print_extrinsics(const rs2_extrinsics& extrinsics, std::ostream& out);
void generate_rdvio_config(const rs2::pipeline_profile& profile, const std::string& output_file);
void processCameraCalibration(const rs2::device& device, 
                             const rs2::pipeline_profile& profile,
                             const rs2::video_stream_profile& ir_stream_left,
                             const rs2::video_stream_profile& ir_stream_right,
                             const rs2::motion_stream_profile& gyro_stream,
                             const rs2::motion_stream_profile& accel_stream,
                             const std::string& output_yaml);

// Function to extract and print intrinsics
void print_intrinsics(const rs2_intrinsics& intrinsics, std::ostream& out) {
    out << std::left << std::setw(14) << "  Width: " << "\t" << intrinsics.width << std::endl
        << std::left << std::setw(14) << "  Height: " << "\t" << intrinsics.height << std::endl
        << std::left << std::setw(14) << "  PPX: " << "\t" << std::setprecision(15) << intrinsics.ppx << std::endl
        << std::left << std::setw(14) << "  PPY: " << "\t" << std::setprecision(15) << intrinsics.ppy << std::endl
        << std::left << std::setw(14) << "  Fx: " << "\t" << std::setprecision(15) << intrinsics.fx << std::endl
        << std::left << std::setw(14) << "  Fy: " << "\t" << std::setprecision(15) << intrinsics.fy << std::endl
        << std::left << std::setw(14) << "  Distortion: " << "\t" << rs2_distortion_to_string(intrinsics.model) << std::endl
        << std::left << std::setw(14) << "  Coeffs: ";

    for (auto i = 0u; i < 5; ++i)
        out << "\t" << std::setprecision(15) << intrinsics.coeffs[i] << "  ";
    out << std::endl;
}

// Function to extract and print motion intrinsics
void print_motion_intrinsics(const rs2_motion_device_intrinsic& intrinsics, std::ostream& out) {
    out << "Bias Variances: \t";
    for (auto i = 0u; i < 3; ++i)
        out << std::setprecision(15) << std::fixed << intrinsics.bias_variances[i] << "  ";

    out << "\nNoise Variances: \t";
    for (auto i = 0u; i < 3; ++i)
        out << std::setprecision(15) << std::fixed << intrinsics.noise_variances[i] << "  ";

    out << "\nSensitivity Matrix: " << std::endl;
    for (auto i = 0u; i < 3; ++i) {
        for (auto j = 0u; j < 3; ++j)
            out << std::right << std::setw(13) << std::setprecision(6) << intrinsics.data[i][j] << "  ";
        out << "\n";
    }
    out << std::endl;
}

// Function to extract and print extrinsics
void print_extrinsics(const rs2_extrinsics& extrinsics, std::ostream& out) {
    out << " Rotation Matrix:\n";
    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            out << std::setw(15) << std::setprecision(10) << extrinsics.rotation[j * 3 + i] << "  ";
        }
        out << std::endl;
    }

    out << "\n Translation Vector: ";
    for (auto i = 0u; i < 3; ++i)
        out << std::setprecision(10) << extrinsics.translation[i] << "  ";
    out << std::endl << std::endl;
}

// Function to generate YAML configuration for rd_vio
void generate_rdvio_config(const rs2::pipeline_profile& profile, const std::string& output_file) {
    try {
        std::ofstream yaml_file(output_file);
        if (!yaml_file.is_open()) {
            std::cerr << "Failed to open file for writing: " << output_file << std::endl;
            return;
        }

        // Get streams from the profile
        auto ir_stream = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>(); // Left IR camera
        auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO).as<rs2::motion_stream_profile>();
        auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::motion_stream_profile>();

        // Get intrinsics
        rs2_intrinsics ir_intrinsics = ir_stream.get_intrinsics();
        rs2_motion_device_intrinsic gyro_intrinsics = gyro_stream.get_motion_intrinsics();
        rs2_motion_device_intrinsic accel_intrinsics = accel_stream.get_motion_intrinsics();

        // Get extrinsics
        rs2_extrinsics ir_to_imu = ir_stream.get_extrinsics_to(gyro_stream);

        // Write YAML
        yaml_file << "%YAML:1.0\n";
        yaml_file << "imu:\n";
        yaml_file << "  # D455 inertial sensor noise model parameters\n";
        
        // Use noise variances from IMU intrinsics
        yaml_file << "  gyroscope_noise_density: " << std::sqrt(gyro_intrinsics.noise_variances[0]) << "     # [ rad / s / sqrt(Hz) ]\n";
        yaml_file << "  gyroscope_random_walk: " << std::sqrt(gyro_intrinsics.bias_variances[0]) << "     # [ rad / s^2 / sqrt(Hz) ]\n";
        yaml_file << "  accelerometer_noise_density: " << std::sqrt(accel_intrinsics.noise_variances[0]) << "  # [ m / s^2 / sqrt(Hz) ]\n";
        yaml_file << "  accelerometer_random_walk: " << std::sqrt(accel_intrinsics.bias_variances[0]) << "  # [ m / s^3 / sqrt(Hz) ]\n";
        yaml_file << "  accelerometer_bias: [0.0, 0.0, 0.0] # acc bias prior\n";
        yaml_file << "  gyroscope_bias: [0.0, 0.0, 0.0]     # gyro bias prior\n";
        
        // Calculate rotation quaternion from extrinsics rotation matrix
        Eigen::Matrix3d rot_matrix;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rot_matrix(i, j) = ir_to_imu.rotation[i * 3 + j];
            }
        }
        Eigen::Quaterniond q(rot_matrix);
        
        yaml_file << "  extrinsic:\n";
        yaml_file << "    q_bi: [ " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << " ] # x y z w\n";
        yaml_file << "    p_bi: [ " << ir_to_imu.translation[0] << ", " 
                               << ir_to_imu.translation[1] << ", " 
                               << ir_to_imu.translation[2] << " ] # x y z [m]\n";
        
        // Calculate IMU noise covariance matrices based on the motion intrinsics
        yaml_file << "  noise:\n";
        yaml_file << "    cov_g: [\n";
        yaml_file << "      " << gyro_intrinsics.noise_variances[0] << ", 0.0, 0.0,\n";
        yaml_file << "      0.0, " << gyro_intrinsics.noise_variances[1] << ", 0.0,\n";
        yaml_file << "      0.0, 0.0, " << gyro_intrinsics.noise_variances[2] << "]\n";
        
        yaml_file << "    cov_a: [\n";
        yaml_file << "      " << accel_intrinsics.noise_variances[0] << ", 0.0, 0.0,\n";
        yaml_file << "      0.0, " << accel_intrinsics.noise_variances[1] << ", 0.0,\n";
        yaml_file << "      0.0, 0.0, " << accel_intrinsics.noise_variances[2] << "]\n";
        
        yaml_file << "    cov_bg: [\n";
        yaml_file << "      " << gyro_intrinsics.bias_variances[0] << ", 0.0, 0.0,\n";
        yaml_file << "      0.0, " << gyro_intrinsics.bias_variances[1] << ", 0.0,\n";
        yaml_file << "      0.0, 0.0, " << gyro_intrinsics.bias_variances[2] << "]\n";
        
        yaml_file << "    cov_ba: [\n";
        yaml_file << "      " << accel_intrinsics.bias_variances[0] << ", 0.0, 0.0,\n";
        yaml_file << "      0.0, " << accel_intrinsics.bias_variances[1] << ", 0.0,\n";
        yaml_file << "      0.0, 0.0, " << accel_intrinsics.bias_variances[2] << "]\n";
        
        yaml_file << "\ncam0:\n";
        yaml_file << "  # D455 infrared camera parameters\n";
        
        // Flip the extrinsics for camera to IMU transformation (inverse of IMU to camera)
        Eigen::Matrix3d rot_matrix_inv = rot_matrix.transpose();
        Eigen::Vector3d trans_vec;
        trans_vec << ir_to_imu.translation[0], ir_to_imu.translation[1], ir_to_imu.translation[2];
        Eigen::Vector3d trans_vec_inv = -rot_matrix_inv * trans_vec;
        
        yaml_file << "  T_BS:\n";
        yaml_file << "    cols: 4\n";
        yaml_file << "    rows: 4\n";
        yaml_file << "    data: [";
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                yaml_file << rot_matrix_inv(i, j) << ", ";
            }
            yaml_file << trans_vec_inv(i) << ", ";
        }
        yaml_file << "0.0, 0.0, 0.0, 1.0]\n";
        
        yaml_file << "  resolution: [" << ir_intrinsics.width << ", " << ir_intrinsics.height << "]\n";
        yaml_file << "  camera_model: pinhole\n";
        yaml_file << "  distortion_model: radtan\n";
        yaml_file << "  intrinsics: [" << ir_intrinsics.fx << ", " << ir_intrinsics.fy << ", " 
                  << ir_intrinsics.ppx << ", " << ir_intrinsics.ppy << "]\n";
        yaml_file << "  camera_distortion_flag: 1\n";
        yaml_file << "  distortion: [" << ir_intrinsics.coeffs[0] << ", " << ir_intrinsics.coeffs[1] << ", " 
                  << ir_intrinsics.coeffs[2] << ", " << ir_intrinsics.coeffs[3] << "]\n";
        yaml_file << "  camera_readout_time: 0.0\n";
        yaml_file << "  time_offset: 0.0\n";
        
        // Camera extrinsics using quaternion representation
        Eigen::Quaterniond q_bc(rot_matrix_inv);
        yaml_file << "  extrinsic:\n";
        yaml_file << "    q_bc: [ " << q_bc.x() << ", " << q_bc.y() << ", " << q_bc.z() << ", " << q_bc.w() << " ]\n";
        yaml_file << "    p_bc: [ " << trans_vec_inv(0) << ", " << trans_vec_inv(1) << ", " << trans_vec_inv(2) << " ]\n";
        
        yaml_file << "  noise: [\n";
        yaml_file << "    0.5, 0.0,\n";
        yaml_file << "    0.0, 0.5]\n";
        
        yaml_file.close();
        std::cout << "Successfully wrote RD-VIO configuration to: " << output_file << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error generating configuration file: " << e.what() << std::endl;
    }
}

// Helper function to process camera calibration
void processCameraCalibration(const rs2::device& device, 
                             const rs2::pipeline_profile& profile,
                             const rs2::video_stream_profile& ir_stream_left,
                             const rs2::video_stream_profile& ir_stream_right,
                             const rs2::motion_stream_profile& gyro_stream,
                             const rs2::motion_stream_profile& accel_stream,
                             const std::string& output_yaml) {
    try {
        // Print device information
        std::cout << "\n=== Device Information ===" << std::endl;
        std::cout << "Camera Name: " << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial Number: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "Firmware Version: " << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
        std::cout << "Physical Port: " << device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;

        // Print intrinsics
        std::cout << "\n=== Left Infrared Camera Intrinsics ===" << std::endl;
        auto ir_left_intrinsics = ir_stream_left.get_intrinsics();
        print_intrinsics(ir_left_intrinsics, std::cout);

        try {
            std::cout << "\n=== Right Infrared Camera Intrinsics ===" << std::endl;
            auto ir_right_intrinsics = ir_stream_right.get_intrinsics();
            print_intrinsics(ir_right_intrinsics, std::cout);
        } catch (const std::exception& e) {
            std::cout << "Right infrared camera not available. Continuing with left camera only." << std::endl;
        }

        std::cout << "\n=== Gyroscope Intrinsics ===" << std::endl;
        auto gyro_intrinsics = gyro_stream.get_motion_intrinsics();
        print_motion_intrinsics(gyro_intrinsics, std::cout);

        std::cout << "\n=== Accelerometer Intrinsics ===" << std::endl;
        auto accel_intrinsics = accel_stream.get_motion_intrinsics();
        print_motion_intrinsics(accel_intrinsics, std::cout);

        // Print extrinsics
        std::cout << "\n=== Extrinsics: Left IR Camera to Gyroscope ===" << std::endl;
        auto ir_left_to_gyro = ir_stream_left.get_extrinsics_to(gyro_stream);
        print_extrinsics(ir_left_to_gyro, std::cout);

        std::cout << "\n=== Extrinsics: Left IR Camera to Accelerometer ===" << std::endl;
        auto ir_left_to_accel = ir_stream_left.get_extrinsics_to(accel_stream);
        print_extrinsics(ir_left_to_accel, std::cout);

        try {
            std::cout << "\n=== Extrinsics: Left IR Camera to Right IR Camera ===" << std::endl;
            auto ir_left_to_ir_right = ir_stream_left.get_extrinsics_to(ir_stream_right);
            print_extrinsics(ir_left_to_ir_right, std::cout);
        } catch (const std::exception& e) {
            std::cout << "Right infrared camera extrinsics not available." << std::endl;
        }

        // Generate RD-VIO configuration file
        std::cout << "\nGenerating RD-VIO configuration file..." << std::endl;
        generate_rdvio_config(profile, output_yaml);
    } catch (const std::exception& e) {
        std::cerr << "\nError during calibration: " << e.what() << std::endl;
        throw; // Rethrow to be caught by the main try-catch
    }
}

int main(int argc, char** argv) {
    try {
        // Get output file path from command line arguments
        std::string output_yaml = "realsense_d455_calibration.yaml";
        if (argc > 1) {
            output_yaml = argv[1];
        }

        // Create RealSense context
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cerr << "No RealSense devices found!" << std::endl;
            std::cerr << "Please make sure your camera is connected and recognized by the system." << std::endl;
            std::cerr << "You can check with 'rs-enumerate-devices' command if it's installed." << std::endl;
            return 1;
        }

        std::cout << "Found " << devices.size() << " RealSense device(s)" << std::endl;
        rs2::device device = devices[0];
        std::cout << "Using device: " << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial Number: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        
        // Check firmware version
        std::string fw_version = device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        std::cout << "Firmware Version: " << fw_version << std::endl;
        
        // Firmware should be newer than 5.13.0.0 for good IMU data
        std::istringstream version_stream(fw_version);
        std::string major_version;
        std::getline(version_stream, major_version, '.');
        
        try {
            int major = std::stoi(major_version);
            if (major < 5) {
                std::cout << "\nWARNING: Your camera firmware might be outdated." << std::endl;
                std::cout << "For optimal results, consider updating to firmware 5.13.0.0 or newer." << std::endl;
                std::cout << "Visit: https://dev.intelrealsense.com/docs/firmware-updates" << std::endl;
            }
        } catch (const std::exception& e) {
            // Ignore parsing errors
        }

        // Configure pipeline
        rs2::pipeline pipe;
        rs2::config cfg;

        // Print available sensors on the device
        std::cout << "\nAvailable sensors on this device:" << std::endl;
        for (auto& sensor : device.query_sensors()) {
            std::cout << "  - " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            
            // Print supported streams for each sensor
            std::cout << "    Supported streams:" << std::endl;
            for (auto& profile : sensor.get_stream_profiles()) {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                if (video_profile) {
                    std::cout << "      - " << rs2_stream_to_string(profile.stream_type()) << " " 
                              << profile.stream_index() << " (" 
                              << video_profile.width() << "x" << video_profile.height() << " @ " 
                              << profile.fps() << " fps, format: " 
                              << rs2_format_to_string(profile.format()) << ")" << std::endl;
                } else if (auto motion_profile = profile.as<rs2::motion_stream_profile>()) {
                    std::cout << "      - " << rs2_stream_to_string(profile.stream_type()) 
                              << " @ " << profile.fps() << " fps" << std::endl;
                }
            }
        }
        
        std::cout << "\nConfiguring streams:" << std::endl;
        
        // Try a more flexible approach with stream configuration
        try {
            // Enable streams needed for calibration - try with default configurations first
            std::cout << "  - Enabling Left IR Stream..." << std::endl;
            cfg.enable_stream(RS2_STREAM_INFRARED, 1);  // Left infrared camera
            
            std::cout << "  - Enabling Right IR Stream..." << std::endl;
            cfg.enable_stream(RS2_STREAM_INFRARED, 2);  // Right infrared camera
            
            std::cout << "  - Enabling Accelerometer..." << std::endl;
            cfg.enable_stream(RS2_STREAM_ACCEL);
            
            std::cout << "  - Enabling Gyroscope..." << std::endl;
            cfg.enable_stream(RS2_STREAM_GYRO);
            
            // Start pipeline with configuration
            std::cout << "\nStarting pipeline to collect calibration information..." << std::endl;
            rs2::pipeline_profile profile = pipe.start(cfg);
            
            // Wait a moment for streams to settle
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Get device streams
            auto ir_stream_left = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>(); // Left IR camera
            auto ir_stream_right = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>(); // Right IR camera
            auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO).as<rs2::motion_stream_profile>();
            auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::motion_stream_profile>();
            
            // Process calibration
            processCameraCalibration(device, profile, ir_stream_left, ir_stream_right, 
                                    gyro_stream, accel_stream, output_yaml);
            
            // Stop the pipeline
            pipe.stop();
            
            std::cout << "\nAutomatic calibration complete. You can now run VIO with:" << std::endl;
            std::cout << "./examples/test_realsense " << output_yaml << " ../configs/setting.yaml" << std::endl;
            
            return 0; // Success
        }
        catch (const rs2::error& e) {
            std::cerr << "\nRealSense error: " << e.what() << std::endl;
            std::cerr << "Error occurred at " << e.get_failed_function() << "(" << e.get_failed_args() << ")" << std::endl;
            
            // Try fallback configuration if your device doesn't support default configuration
            std::cout << "\nTrying fallback configuration..." << std::endl;
            try {
                // Clear configuration
                cfg = rs2::config();
                
                // Try alternative configuration with specific parameters known to work with D455
                std::cout << "  - Enabling Left IR Stream (640x480 @ 30fps)..." << std::endl;
                cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
                
                if (device.get_info(RS2_CAMERA_INFO_NAME) == std::string("Intel RealSense D415")) {
                    std::cout << "  - D415 detected, skipping right IR stream..." << std::endl;
                } else {
                    std::cout << "  - Enabling Right IR Stream (640x480 @ 30fps)..." << std::endl;
                    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
                }
                
                // Try different IMU rates
                std::cout << "  - Enabling Accelerometer (200Hz)..." << std::endl;
                cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
                
                std::cout << "  - Enabling Gyroscope (200Hz)..." << std::endl;
                cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
                
                std::cout << "\nStarting pipeline with fallback configuration..." << std::endl;
                pipe.stop(); // Make sure to stop any previous pipeline
                rs2::pipeline_profile profile = pipe.start(cfg);
                
                // Wait a moment for streams to settle
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // Get device streams
                auto ir_stream_left = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>(); // Left IR camera
                // Get right stream if available, otherwise we'll just use the left for calibration
                rs2::video_stream_profile ir_stream_right(nullptr);
                try {
                    ir_stream_right = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
                } catch(...) {
                    std::cout << "  - Right IR stream not available, using left only" << std::endl;
                }
                
                auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO).as<rs2::motion_stream_profile>();
                auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::motion_stream_profile>();
                
                // Process calibration
                processCameraCalibration(device, profile, ir_stream_left, ir_stream_right, 
                                       gyro_stream, accel_stream, output_yaml);
                
                // Stop the pipeline
                pipe.stop();
                
                std::cout << "\nAutomatic calibration complete. You can now run VIO with:" << std::endl;
                std::cout << "./examples/test_realsense " << output_yaml << " ../configs/setting.yaml" << std::endl;
                
                return 0; // Success
            }
            catch (const rs2::error& e2) {
                std::cerr << "\nFallback configuration also failed: " << e2.what() << std::endl;
                std::cerr << "Error details: " << e2.get_failed_function() << "(" << e2.get_failed_args() << ")" << std::endl;
                std::cerr << "\nPlease check that your camera is properly connected and has firmware version 5.13.0.0 or newer." << std::endl;
                std::cerr << "You can also try disconnecting and reconnecting the camera." << std::endl;
                return 1;
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
    
    return 0;
}