// show_ir_and_imu_slime.cpp ----------------------------------------------------
// build (Linux):
//   g++ show_ir_and_imu_slime.cpp -std=c++17 -pthread \
//       -lrealsense2 `pkg-config --cflags --libs opencv4` -o show_ir_and_imu_slime
// ------------------------------------------------------------------------------

#include <chrono>
#include <iomanip>
#include <iostream>
#include <array>
#include <cmath>
#include <thread>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <rdvio/rdvio.hpp>          // your VIO (if you still want it)
// #include "pviz.hpp"              // optional live viewer
#include "slime.cpp"               // SlimeVR networking helpers
                                    //  – slimeDiscoverServer()
//  – slimeSendHeartbeat()
//  – slimeSendRotationQuat(float x, float y, float z, float w)

/* ---------- helper: simple bias estimator (first 50 gyro samples) ------------ */
struct GyroBias {
    bool   is_set  = false;
    int    n       = 0;
    double min_x=1e3, min_y=1e3, min_z=1e3;
    double max_x=-1e3, max_y=-1e3, max_z=-1e3;
    double x=0, y=0, z=0;

    void update(double gx, double gy, double gz)
    {
        if (n < 50) {                       // collect extrema
            min_x = std::min(min_x, gx);  max_x = std::max(max_x, gx);
            min_y = std::min(min_y, gy);  max_y = std::max(max_y, gy);
            min_z = std::min(min_z, gz);  max_z = std::max(max_z, gz);
            if (++n == 50) {
                x = (max_x + min_x)/2.0;
                y = (max_y + min_y)/2.0;
                z = (max_z + min_z)/2.0;
                is_set = true;
                std::cout << "[IMU] gyro bias set: "
                          << x << ", " << y << ", " << z << std::endl;
            }
        }
    }
};

/* ---------- math helpers ----------------------------------------------------- */
struct Quaternion {
    double x{}, y{}, z{}, w{};
};

static inline Quaternion euler_to_quat(double roll, double pitch, double yaw)
{
    // Z‑YX intrinsic (yaw‑pitch‑roll) – the usual Unity / Slime convention
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

/* ------------------------------ main ----------------------------------------- */
int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file>\n";
        return EXIT_FAILURE;
    }
    const std::string calib_file  = argv[1];
    const std::string config_file = argv[2];

    /* -------- camera & pipeline -------- */
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);   // left IR
    cfg.enable_stream(RS2_STREAM_GYRO ,    RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_ACCEL,    RS2_FORMAT_MOTION_XYZ32F,  63);
    pipe.start(cfg);

    /* -------- warm‑up -------- */
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();

    /* -------- optional VIO -------- */
    rdvio::Odometry vio(calib_file, config_file);

    /* -------- IMU state -------- */
    GyroBias bias;
    std::array<double, RS2_STREAM_COUNT> last_ts{};           // µs
    double roll  = 0.0;   // rad
    double pitch = 0.0;   // rad
    double yaw   = 0.0;   // rad

    auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

    /* -------- SlimeVR: discover & heartbeat -------- */
    bool slime_connected = false;
    while (!slime_connected) {
        std::cout << "[Slime] searching for server …" << std::endl;
        slime_connected = slimeDiscoverServer(0);
        if (!slime_connected)
            std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Slime] connected!" << std::endl;

    std::thread slime_heartbeat_thread([]{
        while (true) {
            slimeSendHeartbeat();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    /* -------- main loop -------- */
    while (true)
    {
        rs2::frameset fs = pipe.wait_for_frames();            // blocking

        /* ----- IR ----- */
        rs2::video_frame ir = fs.get_infrared_frame(1);
        if (ir) {
            double t_img = ir.get_timestamp() * 1e-3;         // ms → s
            cv::Mat img(cv::Size(640, 480), CV_8UC1,
                        (void*)ir.get_data(), cv::Mat::AUTO_STEP);
            vio.addFrame(t_img, img);
            cv::imshow("IR", img);
        }

        /* ----- IMU ----- */
        for (auto &&f : fs)
            if (f.is<rs2::motion_frame>())
            {
                double ts = f.get_timestamp();                // µs
                rs2_vector v = f.as<rs2::motion_frame>().get_motion_data();
                rs2_stream s = f.get_profile().stream_type();

                double dt = (last_ts[s]==0) ? 0
                                             : (ts - last_ts[s]) * 1e-6;      // s
                last_ts[s] = ts;

                if (s == RS2_STREAM_GYRO) {
                    bias.update(v.x, v.y, v.z);
                    if (!bias.is_set || dt == 0) continue;

                    double gx = v.x - bias.x;
                    double gy = v.y - bias.y;
                    double gz = v.z - bias.z;

                    // Integrate (rad)
                    roll  += gz * dt;   // NOTE: device axes → tune if needed
                    pitch += gx * dt;
                    yaw   += gy * dt;
                }
                else if (s == RS2_STREAM_ACCEL && bias.is_set && dt != 0) {
                    /* complementary filter */
                    double ax=v.x, ay=v.y, az=v.z;
                    double acc_roll  = std::atan2( ay,  az);
                    double acc_pitch = std::atan2(-ax,  std::sqrt(ay*ay+az*az));

                    const double alpha = 0.98;                // weight gyro
                    roll  = alpha*roll  + (1-alpha)*acc_roll;
                    pitch = alpha*pitch + (1-alpha)*acc_pitch;
                    /* yaw left to gyro integration only */
                }
            }

        /* ----- pose print & SlimeVR send ----- */
        if (bias.is_set && fs.size()) {
            Quaternion q = euler_to_quat(roll, pitch, yaw);

            // Re‑order / negate to match SlimeVR’s coordinate system
            // (same trick as in your EuRoC example)
            slimeSendRotationQuat(static_cast<float>( q.x),
                                  static_cast<float>( q.z),
                                  static_cast<float>(-q.y),
                                  static_cast<float>( q.w));

            std::cout << std::fixed << std::setprecision(1)
                      << "R=" << rad2deg(roll)  << "°  "
                      << "P=" << rad2deg(pitch) << "°  "
                      << "Y=" << rad2deg(yaw)   << "°\r" << std::flush;
        }

        if (cv::waitKey(1) == 27) break;                      // Esc
    }

    /* -------- cleanup -------- */
    slime_heartbeat_thread.detach();  // let the OS reclaim on exit
    return 0;
}
