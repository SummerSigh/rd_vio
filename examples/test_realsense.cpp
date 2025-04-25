
#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <rdvio/rdvio.hpp>
#include "pviz.hpp"

static void save_camera_calibration(const rs2_intrinsics& K,
                                    const rs2_extrinsics& T_ci,
                                    const std::string&   path)
{
    std::ofstream f(path);
    if(!f){ std::cerr<<"Cannot write "<<path<<'\n'; return; }

    f<<"cam0:\n"
     <<"  camera_model: pinhole\n"
     <<"  intrinsics: ["<<K.fx<<", "<<K.fy<<", "<<K.ppx<<", "<<K.ppy<<"]\n"
     <<"  distortion_model: radtan\n"
     <<"  distortion_coefficients: ["<<K.coeffs[0]<<", "<<K.coeffs[1]<<", "
                                      <<K.coeffs[2]<<", "<<K.coeffs[3]<<", 0.0]\n"
     <<"  resolution: ["<<K.width<<", "<<K.height<<"]\n"
     <<"  T_cam_imu:\n"
     <<"  - ["<<T_ci.rotation[0]<<", "<<T_ci.rotation[1]<<", "<<T_ci.rotation[2]
     <<", "<<T_ci.translation[0]<<"]\n"
     <<"  - ["<<T_ci.rotation[3]<<", "<<T_ci.rotation[4]<<", "<<T_ci.rotation[5]
     <<", "<<T_ci.translation[1]<<"]\n"
     <<"  - ["<<T_ci.rotation[6]<<", "<<T_ci.rotation[7]<<", "<<T_ci.rotation[8]
     <<", "<<T_ci.translation[2]<<"]\n"
     <<"  - [0.0, 0.0, 0.0, 1.0]\n";
}

static std::atomic<bool> running{true};
void sigint_handler(int){ running.store(false); }

struct RealsenseHandler
{
    rdvio::Odometry* vio;
    pviz::Viewer*    viewer;

    void operator()(const rs2::frame& f) const
    {
        try
        {
            if (f.is<rs2::frameset>())            // composite set (colour+depth)
            {
                auto fs    = f.as<rs2::frameset>();
                auto color = fs.get_color_frame();  // may be empty
                if (color)
                {
                    double ts = color.get_timestamp()*1e-3;      // ms → s
                    cv::Mat img(color.get_height(), color.get_width(),
                                CV_8UC3, (void*)color.get_data());
                    img = img.clone();                           // detach
                    vio->addFrame(ts, img);
                    viewer->publish_topic("input", img);
                }
            }
            else if (f.is<rs2::motion_frame>())    // gyro or accel
            {
                auto mf  = f.as<rs2::motion_frame>();
                auto dat = mf.get_motion_data();
                double ts = mf.get_timestamp()*1e-3;

                if (mf.get_profile().stream_type() == RS2_STREAM_GYRO)
                    vio->addGyro(ts,  Eigen::Vector3d(dat.x, dat.y, dat.z));
                else
                    vio->addAcc (ts,  Eigen::Vector3d(dat.x, dat.y, dat.z));
            }
        }
        catch(const std::exception& e) { std::cerr<<"RS callback: "<<e.what()<<'\n'; }
    }
};

int main(int argc,char** argv)
{
    if(argc!=2){
        std::cerr<<"Usage: "<<argv[0]<<" <rdvio_config.yaml>\n";
        return 1;
    }
    std::signal(SIGINT, sigint_handler);

    // --- RealSense ----------------------------------------------------------
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640,480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);

    auto prof = pipe.start(cfg);                       // start once to grab K, T
    auto color_prof = prof.get_stream(RS2_STREAM_COLOR)
                         .as<rs2::video_stream_profile>();
    rs2_intrinsics K = color_prof.get_intrinsics();
    rs2_extrinsics  T_ci = color_prof.get_extrinsics_to(
                                 prof.get_stream(RS2_STREAM_GYRO));

    const std::string calib_file = "realsense_calib.yaml";
    save_camera_calibration(K, T_ci, calib_file);

    pipe.stop();                                      // stop → restart with cb

    rdvio::Odometry vio(calib_file, argv[1]);
    pviz::Settings vs; vs.follow = true;
    pviz::Viewer viewer("realsense", vs);
    std::thread viewer_thread(&pviz::Viewer::run, &viewer);

    // re-start with callback
    RealsenseHandler handler{&vio, &viewer};
    pipe.start(cfg, handler);                         // official API idiom
                                                     // ﻿:contentReference[oaicite:5]{index=5}

    std::cout<<"[INFO] Streaming — press Ctrl-C to exit\n";

    constexpr int RDVIO_TRACKING_STATE = 1;
    while(running.load())
    {
        if(vio.state()==RDVIO_TRACKING_STATE){
            viewer.publish_trajectory(vio.transform_world_cam());
            viewer.publish_local_point_cloud(vio.local_map());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    pipe.stop();
    viewer.exit();
    viewer_thread.join();
    return 0;
}
