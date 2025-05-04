// realsense_d455.cpp
// Build with:
//     g++ realsense_d455.cpp -std=c++17 -lrealsense2 -o realsense_d455
//
// Tested with librealsense 2.55 and Intel RealSense D455 (works the same
// for any RealSense that has Infrared-1 + ACCEL + GYRO streams).

#include <librealsense2/rs.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

using namespace rs2;

// ─────────────────────────────── helpers ───────────────────────────────────
static void rotation_to_quat(const float R[9], double q[4])
{
    double t = R[0] + R[4] + R[8];
    if (t > 0.0) {
        double s = std::sqrt(t + 1.0) * 2.0;
        q[3] = 0.25 * s;
        q[0] = (R[7] - R[5]) / s;
        q[1] = (R[2] - R[6]) / s;
        q[2] = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        q[3] = (R[7] - R[5]) / s;
        q[0] = 0.25 * s;
        q[1] = (R[1] + R[3]) / s;
        q[2] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        q[3] = (R[2] - R[6]) / s;
        q[0] = (R[1] + R[3]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[5] + R[7]) / s;
    } else {
        double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        q[3] = (R[3] - R[1]) / s;
        q[0] = (R[2] + R[6]) / s;
        q[1] = (R[5] + R[7]) / s;
        q[2] = 0.25 * s;
    }
}

// Convert librealsense’s (σ²) arrays → averaged σ (std-dev) per axis
static void get_noise_params(const rs2_motion_device_intrinsic &mi,
                             double &noise_density,
                             double &random_walk)
{
    auto mean_sigma = [](const float v[3]) -> double {
        return (std::sqrt(v[0]) + std::sqrt(v[1]) + std::sqrt(v[2])) / 3.0;
    };
    noise_density = mean_sigma(mi.noise_variances);
    random_walk   = mean_sigma(mi.bias_variances);
}

// Write EuroC-style YAML -----------------------------------------------------
static void write_yaml(const std::string &path,
                       const rs2_intrinsics  &cam_in,
                       const rs2_extrinsics  &ex_cam_from_imu,
                       double cam_rate_hz,
                       double gyro_noise,  double gyro_rw,
                       double accel_noise, double accel_rw,
                       double imu_rate_hz)
{
    double q_bc[4];
    rotation_to_quat(ex_cam_from_imu.rotation, q_bc);

    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open output file");

    out << std::fixed << std::setprecision(8);

    // ── cam0 ───────────────────────────────────────────────────────────
    out << "cam0:\n"
        << "  T_BS:\n"
        << "    cols: 4\n"
        << "    rows: 4\n"
        << "    data: ["
        << ex_cam_from_imu.rotation[0] << ", " << ex_cam_from_imu.rotation[1] << ", "
        << ex_cam_from_imu.rotation[2] << ", " << ex_cam_from_imu.translation[0] << ", "
        << ex_cam_from_imu.rotation[3] << ", " << ex_cam_from_imu.rotation[4] << ", "
        << ex_cam_from_imu.rotation[5] << ", " << ex_cam_from_imu.translation[1] << ", "
        << ex_cam_from_imu.rotation[6] << ", " << ex_cam_from_imu.rotation[7] << ", "
        << ex_cam_from_imu.rotation[8] << ", " << ex_cam_from_imu.translation[2] << ", "
        << "0.0, 0.0, 0.0, 1.0]\n"
        << "  resolution: [" << cam_in.width << ", " << cam_in.height << "]\n"
        << "  camera_model: pinhole\n"
        << "  distortion_model: radtan\n"
        << "  intrinsics: [" << cam_in.fx << ", " << cam_in.fy << ", "
        << cam_in.ppx << ", " << cam_in.ppy << "]\n"
        << "  camera_distortion_flag: 1\n"
        << "  distortion: [" << cam_in.coeffs[0] << ", " << cam_in.coeffs[1] << ", "
                              << cam_in.coeffs[2] << ", " << cam_in.coeffs[3] << "]\n"
        << "  camera_readout_time: 0.0\n"
        << "  time_offset: 0.0\n"
        << "  extrinsic:\n"
        << "    q_bc: [" << q_bc[0] << ", " << q_bc[1] << ", "
                           << q_bc[2] << ", " << q_bc[3] << "]\n"
        << "    p_bc: [" << ex_cam_from_imu.translation[0] << ", "
                           << ex_cam_from_imu.translation[1] << ", "
                           << ex_cam_from_imu.translation[2] << "]\n"
        << "  rate_hz: " << cam_rate_hz << "\n\n";

    // ── imu ────────────────────────────────────────────────────────────
    out << "imu:\n"
        << "  gyroscope_noise_density:     " << gyro_noise  << "\n"
        << "  gyroscope_random_walk:       " << gyro_rw     << "\n"
        << "  accelerometer_noise_density: " << accel_noise << "\n"
        << "  accelerometer_random_walk:   " << accel_rw    << "\n"
        << "  accelerometer_bias: [0.0, 0.0, 0.0]\n"
        << "  gyroscope_bias:     [0.0, 0.0, 0.0]\n"
        << "  extrinsic:\n"
        << "    q_bi: [0.0, 0.0, 0.0, 1.0]\n"
        << "    p_bi: [0.0, 0.0, 0.0]\n"
        << "  rate_hz: " << imu_rate_hz << "\n";

    std::cout << "Wrote rs_sensor.yaml\n";
}

// ──────────────────────────────── main ─────────────────────────────────────
int main()
try
{
    context ctx;
    auto devs = ctx.query_devices();
    if (devs.size() == 0) {
        std::cerr << "No RealSense device detected\n";
        return EXIT_FAILURE;
    }
    device dev = devs.front();

    video_stream_profile cam_profile;  // Infrared-1
    stream_profile      accel_profile; // ACCEL placeholder
    stream_profile      gyro_profile;  // GYRO  placeholder

    // --- find stream profiles ------------------------------------------------
    for (auto &&sensor : dev.query_sensors())
        for (auto &&sp : sensor.get_stream_profiles()) {
            if (!cam_profile  &&
                sp.stream_type() == RS2_STREAM_INFRARED &&
                sp.stream_index() == 1)
                cam_profile = sp.as<video_stream_profile>();

            if (!accel_profile && sp.stream_type() == RS2_STREAM_ACCEL)
                accel_profile = sp;   // keep generic

            if (!gyro_profile  && sp.stream_type() == RS2_STREAM_GYRO)
                gyro_profile  = sp;   // keep generic
        }

    if (!cam_profile)  { std::cerr << "Infrared-1 not found\n";  return EXIT_FAILURE; }
    if (!accel_profile){ std::cerr << "ACCEL stream not found\n"; return EXIT_FAILURE; }
    if (!gyro_profile) { std::cerr << "GYRO stream not found\n";  return EXIT_FAILURE; }

    // --- camera intrinsics / extrinsics -------------------------------------
    rs2_intrinsics cam_in = cam_profile.get_intrinsics();
    rs2_extrinsics ex_cam_from_imu =
        accel_profile.as<motion_stream_profile>().get_extrinsics_to(cam_profile);

    // --- IMU noise parameters ------------------------------------------------
    rs2_motion_device_intrinsic accel_intr =
        accel_profile.as<motion_stream_profile>().get_motion_intrinsics();
    rs2_motion_device_intrinsic gyro_intr  =
        gyro_profile .as<motion_stream_profile>().get_motion_intrinsics();

    double accel_noise, accel_rw;
    double gyro_noise,  gyro_rw;
    get_noise_params(accel_intr, accel_noise, accel_rw);
    get_noise_params(gyro_intr,  gyro_noise,  gyro_rw);

    // --- write YAML ----------------------------------------------------------
    write_yaml("rs_sensor.yaml",
               cam_in, ex_cam_from_imu, cam_profile.fps(),
               gyro_noise, gyro_rw,
               accel_noise, accel_rw,
               accel_profile.fps());          // single rate used by RD-VIO

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error: " << e.what()
              << " (" << e.get_failed_function() << ":" << e.get_failed_args() << ")\n";
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << '\n';
    return EXIT_FAILURE;
}
