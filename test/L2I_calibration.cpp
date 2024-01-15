#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h> 
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <chrono>
#include "common/bag_reader.h"
#include "utils/filter_data.h"
#include "common/pcd_grid.h"
#include "common/config.h"
#include "L2I_calibration/calibration.h"

int main(int argc, char** argv)
{
    // ros::init (argc, argv, "bag_reader");
    // rosbag::Bag bag;

    std::string config_path = "/media/data/L2I_calibration/config/config.cfg";
    Config cfg(config_path);

    L2I_Calibration::Ptr l2i_calibration(new L2I_Calibration(cfg));
    l2i_calibration -> Init();
    auto start_inter_pose = std::chrono::high_resolution_clock::now();
    std::map<double, Eigen::Matrix4d> T = l2i_calibration -> Run2();
    // std::cout << T.size() << std::endl;
    auto end_inter_pose = std::chrono::high_resolution_clock::now();
    auto duration_inter_pose = std::chrono::duration_cast<std::chrono::microseconds>(end_inter_pose - start_inter_pose);
    // std::cout << "time of inter pose and lidar undistort: " << duration_inter_pose.count() / 1000 << " ms." << std::endl;
    int grid_size = cfg.getvalue<int>("grid_size");

}
         