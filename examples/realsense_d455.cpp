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
            return 1;
        }

        std::cout << "Found " << devices.size() << " RealSense device(s)" << std::endl;
        rs2::device device = devices[0];
        std::cout << "Using device: " << device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial Number: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

        // Configure pipeline
        rs2::pipeline pipe;
        rs2::config cfg;

        // Enable streams needed for calibration
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30); // Left infrared camera
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30); // Right infrared camera (for reference)
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);

        // Start pipeline with configuration
        std::cout << "Starting pipeline to collect calibration information..." << std::endl;
        rs2::pipeline_profile profile = pipe.start(cfg);

        // Wait a moment for streams to settle
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Get device streams
        auto ir_stream_left = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>(); // Left IR camera
        auto ir_stream_right = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>(); // Right IR camera
        auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO).as<rs2::motion_stream_profile>();
        auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::motion_stream_profile>();

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

        std::cout << "\n=== Right Infrared Camera Intrinsics ===" << std::endl;
        auto ir_right_intrinsics = ir_stream_right.get_intrinsics();
        print_intrinsics(ir_right_intrinsics, std::cout);

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

        std::cout << "\n=== Extrinsics: Left IR Camera to Right IR Camera ===" << std::endl;
        auto ir_left_to_ir_right = ir_stream_left.get_extrinsics_to(ir_stream_right);
        print_extrinsics(ir_left_to_ir_right, std::cout);

        // Generate RD-VIO configuration file
        std::cout << "\nGenerating RD-VIO configuration file..." << std::endl;
        generate_rdvio_config(profile, output_yaml);
        
        // Stop the pipeline
        pipe.stop();

        std::cout << "\nAutomatic calibration complete. You can now run VIO with:" << std::endl;
        std::cout << "./examples/test_realsense " << output_yaml << " ../configs/setting.yaml" << std::endl;

        return 0;
    }
    catch (const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}