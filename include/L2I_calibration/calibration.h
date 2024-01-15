#pragma once

#include <iostream>
#include "common/bag_reader.h"
#include "common/config.h"
#include "common/pcd_grid.h"

class L2I_Calibration
{
public:
    typedef std::shared_ptr<L2I_Calibration> Ptr;
    explicit L2I_Calibration(const Config& cfg);
    ~L2I_Calibration() = default;
    
    bool Init();
    std::map<double, Eigen::Matrix4d> Run();
    std::map<double, Eigen::Matrix4d> Run2();

    PointCloud::Ptr FilterLidar(pcl::PointCloud<RsPointXYZIRT_f>::Ptr &cloud,
                        pcl::PointCloud<RsPointXYZIRT_f>::Ptr &pc_in,
                        Eigen::Matrix4d T, std::string path);
    Eigen::Matrix4d getCurTimePose(double cur_time);



private:
    std::string map_path_;
    std::string output_path_;
    BagReader bag_reader_;
    Config cfg_;
    PCDGrid pcd_grid_;
    std::string pose_path_;
    std::string map_las_path_;
    double filter_lidar_pts_threshold_x_;
    double filter_lidar_pts_threshold_y_;
    double filter_lidar_pts_threshold_z_;

    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> lidar_msg_;
    std::map<double, Eigen::Matrix4d> gps_msg_;
    std::vector<Eigen::Matrix4d> l2i_vec_;
    int down_samples_;
};