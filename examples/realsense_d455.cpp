#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rdvio/rdvio.hpp>
#include "slime.cpp"

struct ImgSample {
    double  ts_sec {0.0};
    cv::Mat img;
};

struct ImuSamplePair {
    double          ts_sec {0.0};
    Eigen::Vector3d accel  {0,0,0};
    Eigen::Vector3d gyro   {0,0,0};
};

std::shared_ptr<ImgSample>      g_latest_img;
std::shared_ptr<ImuSamplePair>  g_latest_imu;
std::mutex                      g_img_mtx, g_imu_mtx;

template<typename T>
inline void swap_into(std::shared_ptr<T>& slot, std::shared_ptr<T>&& val,
                      std::mutex& mtx)
{
    std::lock_guard<std::mutex> lock(mtx);
    slot.swap(val);
}

template<typename T>
inline std::shared_ptr<T> take_latest(std::shared_ptr<T>& slot,
                                      std::mutex& mtx)
{
    std::lock_guard<std::mutex> lock(mtx);
    std::shared_ptr<T> out;
    slot.swap(out);
    return out;
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <calib_file.yml> <config_file.yaml>\n";
        return EXIT_FAILURE;
    }
    const std::string calib_file  = argv[1];
    const std::string config_file = argv[2];

    // RealSense initialisation
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_GYRO ,    RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_ACCEL,    RS2_FORMAT_MOTION_XYZ32F,  63);

    rs2::pipeline_profile profile = pipe.start(cfg);

    // disable IR emitter and set the internal queue to 2 frames
    for (auto& s : profile.get_device().query_sensors()) {
        if (s.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
            s.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
        if (s.supports(RS2_OPTION_EMITTER_ENABLED))
            s.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        if (s.supports(RS2_OPTION_LASER_POWER))
            s.set_option(RS2_OPTION_LASER_POWER, 0.f);
    }

    // VIO initialisation
    rdvio::Odometry vio(calib_file, config_file);

    // SlimeVR server stuff
    while (!slimeDiscoverServer(0)) {
        std::cout << "[Slime] searching for server …\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Slime] connected!\n";
    std::thread([]{
        for (;;) { slimeSendHeartbeat(); std::this_thread::sleep_for(
                       std::chrono::seconds(1)); }
    }).detach();
    
    // RealSense outputs in ms and RD-VIO needs seconds for some reason
    constexpr double TS_SCALE = 1e-3;


    std::thread cap_thread([&]{
        rs2::frameset fs;
        ImuSamplePair imu_pair; //RD-VIO expects the data in pairs so we can only run the IMU data at the rate of the slowest compenent. I should figure out a way to cahnge that
        bool have_accel = false, have_gyro = false;

        while (true) {
            if (!pipe.poll_for_frames(&fs)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            //pull IR image from realsense stream
            if (auto ir = fs.get_infrared_frame(1)) {
                auto sample   = std::make_shared<ImgSample>();
                sample->ts_sec = ir.get_timestamp() * TS_SCALE;
                sample->img    = cv::Mat(cv::Size(1280,720), CV_8UC1,
                                         (void*)ir.get_data(),
                                         cv::Mat::AUTO_STEP)
                                     .clone();          // own copy
                swap_into(g_latest_img, std::move(sample), g_img_mtx);
            }

            //pull IMU from realsense stream
            for (auto&& f : fs) {
                if (!f.is<rs2::motion_frame>()) continue;
                const double ts_sec = f.get_timestamp() * TS_SCALE;
                rs2_stream stype    = f.get_profile().stream_type();
                rs2_vector v        = f.as<rs2::motion_frame>().get_motion_data();

                imu_pair.ts_sec = ts_sec;
                if (stype == RS2_STREAM_ACCEL) {
                    imu_pair.accel = {v.x, v.y, v.z};
                    have_accel = true;
                } else if (stype == RS2_STREAM_GYRO) {
                    imu_pair.gyro  = {v.x, v.y, v.z};
                    have_gyro = true;
                }

                if (have_accel && have_gyro) {
                    swap_into(g_latest_imu,
                              std::make_shared<ImuSamplePair>(imu_pair),
                              g_imu_mtx);
                    have_accel = have_gyro = false;
                }
            }
        }
    });

    //We calibrate the IMU bias here so you have to leave your realsense down and let it calibrate bias. Im not sure if this helps but i did this when trying to debug and just left it here. 
    std::thread proc_thread([&]{
        constexpr int   BIAS_SAMPLES = 500;
        bool            bias_done = false;
        int             cnt_a = 0, cnt_g = 0;
        Eigen::Vector3d sum_a(0,0,0), sum_g(0,0,0);
        Eigen::Vector3d accel_bias(0,0,0), gyro_bias(0,0,0);

        for (;;) { //no idea you could do this in C++
            auto img = take_latest(g_latest_img, g_img_mtx);
            auto imu = take_latest(g_latest_imu, g_imu_mtx);

            if (!imu) { std::this_thread::sleep_for(
                            std::chrono::milliseconds(1)); continue; }

            if (!bias_done) {
                sum_a += imu->accel; ++cnt_a;
                sum_g += imu->gyro ; ++cnt_g;
                if (cnt_a >= BIAS_SAMPLES && cnt_g >= BIAS_SAMPLES) {
                    accel_bias = sum_a / cnt_a;
                    gyro_bias  = sum_g / cnt_g;
                    bias_done  = true;
                    std::cout << "\n[IMU] Bias calibration finished\n"
                              << "     Gyro  bias:  " << gyro_bias.transpose()  << "\n"
                              << "     Accel bias: " << accel_bias.transpose() << "\n";
                }
                continue;
            }

            vio.addMotion(imu->ts_sec,
                          imu->accel - accel_bias,
                          imu->gyro  - gyro_bias);

            if (img)
                vio.addFrame(img->ts_sec, img->img);

            if (vio.state() == 1) {
                Eigen::Matrix4d T_wc = vio.transform_world_cam();
                Eigen::Quaterniond q (T_wc.block<3,3>(0,0));

                slimeSendRotationQuat(static_cast<float>( q.x()),
                                      static_cast<float>( q.z()),
                                      static_cast<float>(-q.y()),
                                      static_cast<float>( q.w()));

                std::cout << std::fixed << std::setprecision(3)
                          << "\rQuat: [" << q.x() << ", " << q.z()
                          << ", " << -q.y() << ", " << q.w() << "]   "
                          << std::flush;
            }
        }
    });

    cap_thread.join();
    proc_thread.join();
    return 0;
}
