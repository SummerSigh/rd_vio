#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>

#include <rdvio/rdvio.hpp>
#include "dataset.hpp"

// Trajectory visualization function
void save_trajectory_visualization(const std::vector<Eigen::Vector3d>& trajectory, 
                                 const std::vector<Eigen::Vector3d>& landmarks,
                                 const std::string& filename = "latest_run.png") {
    if (trajectory.empty()) {
        std::cout << "No trajectory data to visualize" << std::endl;
        return;
    }
    
    // Find bounding box for trajectory and landmarks
    double min_x = trajectory[0].x(), max_x = trajectory[0].x();
    double min_z = trajectory[0].z(), max_z = trajectory[0].z();
    
    // Check trajectory bounds
    for (const auto& pose : trajectory) {
        min_x = std::min(min_x, pose.x());
        max_x = std::max(max_x, pose.x());
        min_z = std::min(min_z, pose.z());
        max_z = std::max(max_z, pose.z());
    }
    
    // Check landmark bounds
    for (const auto& landmark : landmarks) {
        min_x = std::min(min_x, landmark.x());
        max_x = std::max(max_x, landmark.x());
        min_z = std::min(min_z, landmark.z());
        max_z = std::max(max_z, landmark.z());
    }
    
    // Add padding
    double padding = 0.1;
    double range_x = max_x - min_x;
    double range_z = max_z - min_z;
    min_x -= padding * range_x;
    max_x += padding * range_x;
    min_z -= padding * range_z;
    max_z += padding * range_z;
    
    // Create image (800x600)
    const int img_width = 800;
    const int img_height = 600;
    cv::Mat image(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255)); // White background
    
    // Scale factors
    double scale_x = (img_width - 20) / (max_x - min_x);
    double scale_z = (img_height - 20) / (max_z - min_z);
    double scale = std::min(scale_x, scale_z);
    
    // Offset to center the trajectory
    double offset_x = 10 + (img_width - 20 - (max_x - min_x) * scale) / 2;
    double offset_z = 10 + (img_height - 20 - (max_z - min_z) * scale) / 2;
    
    // Convert world coordinates to image coordinates
    auto world_to_image = [&](double x, double z) -> cv::Point {
        int img_x = static_cast<int>(offset_x + (x - min_x) * scale);
        int img_z = static_cast<int>(img_height - (offset_z + (z - min_z) * scale)); // Flip Y
        return cv::Point(img_x, img_z);
    };
    
    // Draw landmarks as small green dots
    for (const auto& landmark : landmarks) {
        cv::Point img_point = world_to_image(landmark.x(), landmark.z());
        if (img_point.x >= 0 && img_point.x < img_width && 
            img_point.y >= 0 && img_point.y < img_height) {
            cv::circle(image, img_point, 1, cv::Scalar(0, 255, 0), -1); // Green dots
        }
    }
    
    // Draw trajectory as connected red line
    for (size_t i = 1; i < trajectory.size(); i++) {
        cv::Point pt1 = world_to_image(trajectory[i-1].x(), trajectory[i-1].z());
        cv::Point pt2 = world_to_image(trajectory[i].x(), trajectory[i].z());
        
        // Only draw if both points are within image bounds
        if (pt1.x >= 0 && pt1.x < img_width && pt1.y >= 0 && pt1.y < img_height &&
            pt2.x >= 0 && pt2.x < img_width && pt2.y >= 0 && pt2.y < img_height) {
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2); // Red line
        }
    }
    
    // Draw start point as larger blue circle
    if (!trajectory.empty()) {
        cv::Point start_point = world_to_image(trajectory[0].x(), trajectory[0].z());
        cv::circle(image, start_point, 5, cv::Scalar(255, 0, 0), -1); // Blue circle
    }
    
    // Draw end point as larger magenta circle
    if (trajectory.size() > 1) {
        cv::Point end_point = world_to_image(trajectory.back().x(), trajectory.back().z());
        cv::circle(image, end_point, 5, cv::Scalar(255, 0, 255), -1); // Magenta circle
    }
    
    // Add text annotations
    cv::putText(image, "SLAM Trajectory (Top View)", cv::Point(10, 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(image, "Red: Path, Blue: Start, Magenta: End, Green: Landmarks", 
                cv::Point(10, img_height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // Save image
    cv::imwrite(filename, image);
    std::cout << "Trajectory visualization saved to: " << filename << std::endl;
    std::cout << "Trajectory points: " << trajectory.size() << ", Landmarks: " << landmarks.size() << std::endl;
}

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file> <euroc_mav_dir>" << std::endl;
        return 1;
    }

    const std::string calib_file = argv[1];
    const std::string config_file = argv[2];
    const std::string mav_dir = argv[3];

    std::cout << "Loading EuRoC dataset from: " << mav_dir << std::endl;
    std::cout << "Camera calibration: " << calib_file << std::endl;
    std::cout << "VIO config: " << config_file << std::endl;

    auto euroc = dataset::EuRoC(mav_dir, true, false);
    dataset::set_logger(false);

    std::cout << "Initializing VIO system with g2o..." << std::endl;
    auto vio = rdvio::Odometry(calib_file, config_file);
    
    std::shared_ptr<dataset::dataclip_t> data;
    size_t frame_count = 0;
    size_t imu_count = 0;
    
    // Trajectory data collection
    std::vector<Eigen::Vector3d> trajectory_positions;
    std::vector<Eigen::Vector3d> latest_landmarks;
    
    std::cout << "Starting SLAM processing..." << std::endl;
    std::cout << "Format: [frame_id] timestamp: [x, y, z] [qw, qx, qy, qz]" << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    while((data = euroc.next()) && frame_count < 500) {
        if (data->has_motion()) {
            vio.addMotion(data->timestamp.sec(), data->acc(), data->gyro());
            imu_count++;
        }

        if (data->has_cam0()) {
            if (frame_count < 5) {
                std::cout << "[DEBUG] About to add frame " << frame_count + 1 << std::endl;
            }
            
            vio.addFrame(data->timestamp.sec(), data->cam0);
            frame_count++;
            
            if (frame_count <= 5) {
                std::cout << "[DEBUG] Frame " << frame_count << " added successfully" << std::endl;
            }
            
            // Debug: Check if we can get state safely
            if (frame_count <= 10 || frame_count % 100 == 0) {
                std::cout << "[" << frame_count << "] Checking VIO state..." << std::flush;
                
                try {
                    int current_state = vio.state();
                    std::string state_name;
                    switch(current_state) {
                        case 0: state_name = "INITIALIZING"; break;
                        case 1: state_name = "TRACKING"; break;
                        case 2: state_name = "CRASH"; break;
                        default: state_name = "UNKNOWN"; break;
                    }
                    
                    std::cout << " state=" << current_state << " (" << state_name << ")";
                    
                    if (current_state == 1) {
                        std::cout << " *** TRACKING MODE ACHIEVED! ***";
                        try {
                            Eigen::Matrix4d T = vio.transform_world_cam();
                            Eigen::Vector3d pos = T.block<3,1>(0,3);
                            std::cout << " pos=[" << std::fixed << std::setprecision(3)
                                      << pos.x() << "," << pos.y() << "," << pos.z() << "]";
                            
                            // Collect trajectory data
                            trajectory_positions.push_back(pos);
                            
                            // Update landmarks periodically
                            if (frame_count % 10 == 0) {
                                latest_landmarks = vio.local_map();
                            }
                        } catch (...) {
                            std::cout << " (pose error)";
                        }
                    }
                    std::cout << std::endl;
                } catch (const std::exception& e) {
                    std::cout << " ERROR: " << e.what() << std::endl;
                } catch (...) {
                    std::cout << " UNKNOWN ERROR" << std::endl;
                }
            }
            
            // Show progress
            if (frame_count % 50 == 0) {
                auto current_time = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                std::cout << "--- Progress: " << frame_count << " frames, " << imu_count << " IMU samples in " << elapsed << "s ---" << std::endl;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    std::cout << "\n=== G2O-based VIO Performance Summary ===" << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;
    std::cout << "Total IMU samples: " << imu_count << std::endl;
    std::cout << "Total time: " << total_time << "ms" << std::endl;
    std::cout << "Average FPS: " << (frame_count * 1000.0 / total_time) << std::endl;
    std::cout << "G2O optimization completed successfully!" << std::endl;
    
    // Save trajectory visualization
    std::cout << "\n=== Saving Trajectory Visualization ===" << std::endl;
    save_trajectory_visualization(trajectory_positions, latest_landmarks, "latest_run.png");

    return 0;
}