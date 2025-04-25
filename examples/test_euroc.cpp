#include <chrono>

#include <rdvio/rdvio.hpp>
#include "dataset.hpp"
#include "pviz.hpp"
#include "slime.cpp"


int main(int argc, char** argv) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file> <euroc_mav_dir>" << std::endl;
        return 1;
    }

    const std::string calib_file = argv[1];
    const std::string config_file = argv[2];
    const std::string mav_dir = argv[3];

    auto euroc = dataset::EuRoC(mav_dir, true, false);
    dataset::set_logger(false);

    auto vio = rdvio::Odometry(calib_file, config_file);
    
    auto viewer_setting = pviz::Settings();
    viewer_setting.follow = true;
    auto viewer = pviz::Viewer("euroc", viewer_setting);

    std::thread viewer_thread(&pviz::Viewer::run, &viewer);

    // Initialize SlimeVR connection with retries
    bool slime_connected = false;
    while(!slime_connected) {
        std::cout << "Searching for SlimeVR server..." << std::endl;
        slime_connected = slimeDiscoverServer(0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Start heartbeat thread
    std::thread slime_heartbeat_thread([]() {
        while(true) {
            slimeSendHeartbeat();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    std::shared_ptr<dataset::dataclip_t> data;
    
    // Variables to track processing frequency
    auto last_motion_timestamp = std::chrono::high_resolution_clock::now();
    auto last_visual_timestamp = std::chrono::high_resolution_clock::now();
    double motion_total_hz = 0.0;
    double visual_total_hz = 0.0;
    int motion_count = 0;
    int visual_count = 0;

    while((data = euroc.next())) {

        if (data->has_motion()) {        
            auto start = std::chrono::high_resolution_clock::now();
            
            vio.addMotion(data->timestamp.sec(), data->acc(), data->gyro());

#ifndef USE_MULTI_THREADING
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed > 0)
                std::cout << "[rdvio] motion tracking takes " << elapsed << "ms" << std::endl;
            
            // Calculate and display motion tracking frequency
            auto current_time = std::chrono::high_resolution_clock::now();
            double delta_sec = std::chrono::duration<double>(current_time - last_motion_timestamp).count();
            if (delta_sec > 0) {
                double hz = 1.0 / delta_sec;
                motion_total_hz += hz;
                motion_count++;
                std::cout << "[rdvio] motion tracking rate: " << hz << " Hz (avg: " << (motion_total_hz/motion_count) << " Hz)" << std::endl;
                last_motion_timestamp = current_time;
            }
#else
            // Update timestamp for frequency calculation in multi-threading mode
            auto current_time = std::chrono::high_resolution_clock::now();
            double delta_sec = std::chrono::duration<double>(current_time - last_motion_timestamp).count();
            if (delta_sec > 0) {
                double hz = 1.0 / delta_sec;
                motion_total_hz += hz;
                motion_count++;
                std::cout << "[rdvio] motion tracking rate: " << hz << " Hz (avg: " << (motion_total_hz/motion_count) << " Hz)" << std::endl;
                last_motion_timestamp = current_time;
            }
            // Reduced sleep time for faster motion processing
           // std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif            
        }

        if (data->has_cam0()) {
            auto start = std::chrono::high_resolution_clock::now();

            vio.addFrame(data->timestamp.sec(), data->cam0);
            viewer.publish_topic("input", data->cam0);

#ifndef USE_MULTI_THREADING
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed > 0)
                std::cout << "[rdvio] visual tracking takes " << elapsed << "ms" << std::endl;
            
            // Calculate and display visual tracking frequency
            auto current_time = std::chrono::high_resolution_clock::now();
            double delta_sec = std::chrono::duration<double>(current_time - last_visual_timestamp).count();
            if (delta_sec > 0) {
                double hz = 1.0 / delta_sec;
                visual_total_hz += hz;
                visual_count++;
                std::cout << "[rdvio] visual tracking rate: " << hz << " Hz (avg: " << (visual_total_hz/visual_count) << " Hz)" << std::endl;
                last_visual_timestamp = current_time;
            }
#else
            // Update timestamp for frequency calculation in multi-threading mode
            auto current_time = std::chrono::high_resolution_clock::now();
            double delta_sec = std::chrono::duration<double>(current_time - last_visual_timestamp).count();
            if (delta_sec > 0) {
                double hz = 1.0 / delta_sec;
                visual_total_hz += hz;
                visual_count++;
                std::cout << "[rdvio] visual tracking rate: " << hz << " Hz (avg: " << (visual_total_hz/visual_count) << " Hz)" << std::endl;
                last_visual_timestamp = current_time;
            }
            // Reduced sleep time for faster visual processing
           // std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif            
        }

        if(vio.state() == 1) {
            
            // Extract and adjust rotation for SlimeVR coordinate system
            Eigen::Matrix4d T = vio.transform_world_cam();
            Eigen::Matrix3d R = T.block<3,3>(0,0);
            Eigen::Quaterniond q(R);
            
            std::cout << "Rotation Quaternion: [" << q.x() << ", " << q.z() << ", " << -q.y() << ", " << q.w() << "]\n";

            // Convert to SlimeVR's coordinate system (adjust if needed)
            slimeSendRotationQuat(q.x(), q.z(), -q.y(), q.w());
            
            viewer.publish_local_point_cloud(vio.local_map());
        }
    }

    cv::waitKey(0);

    viewer.exit();
    viewer_thread.join();

    return 0;
}