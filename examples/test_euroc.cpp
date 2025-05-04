// show_ir_and_imu_slime.cpp ----------------------------------------------------
// build (Linux):
//   g++ show_ir_and_imu_slime.cpp -std=c++17 -pthread \
//       -lrealsense2 `pkg-config --cflags --libs opencv4 eigen3` -o show_ir_and_imu_slime
// ------------------------------------------------------------------------------

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rdvio/rdvio.hpp>            // visual‑inertial odometry
#include "slime.cpp"                  // SlimeVR helpers  (discover / heartbeat / quat send)

// ───────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file>\n";
        return EXIT_FAILURE;
    }
    const std::string calib_file  = argv[1];
    const std::string config_file = argv[2];

    // ── RealSense: IR + IMU streams ───────────────────────────────────────────
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);  // left IR
    cfg.enable_stream(RS2_STREAM_GYRO ,    RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_ACCEL,    RS2_FORMAT_MOTION_XYZ32F,  63);

    rs2::pipeline_profile profile = pipe.start(cfg);

    for (auto& s : profile.get_device().query_sensors()) {
        if (s.supports(RS2_OPTION_EMITTER_ENABLED))
            s.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        if (s.supports(RS2_OPTION_LASER_POWER))
            s.set_option(RS2_OPTION_LASER_POWER, 0.f);
    }

    // ── warm‑up ───────────────────────────────────────────────────────────────
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();

    // ── rdvio ­────────────────────────────────────────────────────────────────
    rdvio::Odometry vio(calib_file, config_file);

    // ── SlimeVR discovery + heartbeat ────────────────────────────────────────
    while (!slimeDiscoverServer(0)) {
        std::cout << "[Slime] searching for server …\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Slime] connected!\n";

    std::thread slime_heartbeat_thread([]{
        while (true) {
            slimeSendHeartbeat();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    slime_heartbeat_thread.detach();

    // ── IMU sample buffering (RealSense delivers gyro & accel separately) ────
    struct ImuSample {
        double                       ts_sec{0.0};
        Eigen::Vector3d              vec{0,0,0};
        bool                         valid{false};
    };
    ImuSample last_accel, last_gyro;
    std::mutex imu_mtx;               // protect shared IMU state (not strictly
                                      // necessary in single‑thread capture, but
                                      // ready for threaded variants)

    // ── main loop ─────────────────────────────────────────────────────────────
    while (true)
    {
        rs2::frameset fs = pipe.wait_for_frames();   // blocking

        /* ── image frame ──────────────────────────────────────────────────── */
        if (auto ir = fs.get_infrared_frame(1)) {
            double t_img = ir.get_timestamp() * 1e-6;              // µs → s
            cv::Mat img(cv::Size(640,480), CV_8UC1,
                        (void*)ir.get_data(), cv::Mat::AUTO_STEP);

            vio.addFrame(t_img, img);
            cv::imshow("IR", img);
        }

        /* ── IMU frames ───────────────────────────────────────────────────── */
        for (auto&& f : fs) {
            if (!f.is<rs2::motion_frame>()) continue;

            const double ts_sec = f.get_timestamp() * 1e-6;        // µs → s
            rs2_stream    stype  = f.get_profile().stream_type();
            rs2_vector    v      = f.as<rs2::motion_frame>().get_motion_data();

            std::lock_guard<std::mutex> lock(imu_mtx);
            if (stype == RS2_STREAM_GYRO) {
                last_gyro.ts_sec = ts_sec;
                last_gyro.vec    = {v.x, v.y, v.z};
                last_gyro.valid  = true;
            }
            else if (stype == RS2_STREAM_ACCEL) {
                last_accel.ts_sec = ts_sec;
                last_accel.vec    = {v.x, v.y, v.z};
                last_accel.valid  = true;
            }

            // whenever we have *both* a gyro & accel sample, feed one packet
            if (last_gyro.valid && last_accel.valid) {
                // choose the newer of the two timestamps
                double t_imu = std::max(last_gyro.ts_sec, last_accel.ts_sec);
                vio.addMotion(t_imu, last_accel.vec, last_gyro.vec);

                // consume the samples
                last_gyro.valid  = false;
                last_accel.valid = false;
            }
        }

        /* ── once VIO is tracking, send pose to SlimeVR ──────────────────── */
        if (vio.state() == 1) {
            Eigen::Matrix4d T_wc = vio.transform_world_cam();   // 4×4
            Eigen::Matrix3d R    = T_wc.block<3,3>(0,0);
            Eigen::Quaterniond q(R);

            // reorder axes to match SlimeVR coordinate system
            slimeSendRotationQuat(static_cast<float>( q.x()),
                                  static_cast<float>( q.z()),
                                  static_cast<float>(-q.y()),
                                  static_cast<float>( q.w()));

            std::cout << std::fixed << std::setprecision(3)
                      << "\rQuat: [" << q.x() << ", " << q.z()
                      << ", " << -q.y() << ", " << q.w() << "]   " << std::flush;
        }

        /* escape key closes the viewer */
        if (cv::waitKey(1) == 27) break;
    }
    return 0;
}
