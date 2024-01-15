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

#include "common/bag_reader.h"
#include "utils/filter_data.h"
#include "L2I_calibration/cloud_icp.h"
#include "common/pcd_grid.h"
#include "common/config.h"
#include "L2I_calibration/calibration.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "bag_reader");
    rosbag::Bag bag;

    std::string config_path = "/media/data/L2I_calibration/config/config.cfg";
    Config cfg(config_path);

    // L2I_Calibration::Ptr l2i_calibration(new L2I_Calibration(cfg));
    // l2i_calibration -> Init();
    // l2i_calibration -> Run();
    // std::string config_path = "/media/data/L2I_calibration/config/config.cfg";
    // Config cfg(config_path);

    int grid_size = cfg.getvalue<int>("grid_size");
    std::string map_path = cfg.getvalue<std::string>("map_path");
    std::string las_path = cfg.getvalue<std::string>("las_path");
    std::string output_path = cfg.getvalue<std::string>("output_path");
    std::string bag_path = cfg.getvalue<std::string>("bag_path");
    double ts_begin = cfg.getvalue<double>("ts_begin");
    double ts_end = cfg.getvalue<double>("ts_end");
    
    PCDGrid pcd_grid(las_path, grid_size);
    pcd_grid.LoadLasCloud(las_path, map_path);
    std::vector<std::string> topics; 
     topics.push_back(std::string("/met/gps/lla"));         
    // topics.push_back(std::string("/met/lidar"));      
    topics.push_back(std::string("/points_raw"));      
    BagReader bag_reader(bag_path, ts_begin, ts_end, topics,bag_path);
    bag_reader.DataParse();
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> toplidar_msg = bag_reader.get_toplidar_msg();
    std::map<double, Eigen::Matrix4d> gps_msg = bag_reader.get_gps_msg();
    

    Eigen::Vector3d translation(374974.33, 3447133.99, 32 );
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr point_in(new pcl::PointCloud<RsPointXYZIRT_f>());
    std::vector<std::string> sub_map_path;
    pcd_grid.GetLocalMap(las_path + "map/", translation, point_in);
    // sub_map_path = pcd_grid.GetBlockIDByFileName(x,y,z);
    
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> msg = bag_reader.get_toplidar_msg();
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr>::iterator it = msg.begin();
    double time_cur = it->first;
    // save_lidar_msg_to_pcd(time_cur, *(it->second), pcd_save_path);
    std::cout << msg.size() << std::endl;

}
         