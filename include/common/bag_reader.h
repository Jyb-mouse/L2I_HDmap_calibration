#pragma once

#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>


struct RsPointXYZIRT_f {
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT_f,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity,
                                                                          intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))
class BagReader
{    
 public:
    typedef std::shared_ptr<BagReader> Ptr;
    BagReader(std::string bag_path, double ts_begin, double ts_end, std::vector<std::string> topics, std::string pose_path);
    BagReader() = default;
    ~BagReader();
    void Init(std::string bag_path, double ts_begin, double ts_end, std::vector<std::string> topics, std::string pose_path);
    void DataParse();
    // bool Reset(double ts_begin = -1, double ts_end = -1, std::vector<std::string> hard_topics = {},
    //          std::vector<std::string> soft_topics = {});
    // bool Reset(std::vector<std::string> hard_topics = {}, std::vector<std::string> soft_topics = {});
    // bool Next();

    inline std::unordered_map<std::string, std::vector<rosbag::MessageInstance>> get_msgs() { return msgs_; }

    inline std::string get_bag_path() const { return bag_path_;}
    inline double get_ts_begin() const { return ts_begin_;}
    inline double get_ts_end() const { return ts_end_;}
    inline std::vector<std::string> get_topics() const {return topics_;}

    inline std::map<double, Eigen::Matrix4d> get_gps_msg() const {return gps_msg_;}
    inline std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> get_lidar_msg() const {return lidar_msg_;}
    inline std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> get_toplidar_msg() const {return toplidar_msg_;}

    std::pair<double, Eigen::Matrix4d> GetNearestTimePose(double t);

    void txt_to_queue(std::string filename);

 private:

    void GpsDataTran(const sensor_msgs::NavSatFix::ConstPtr p);
    void LidarDataTran(const sensor_msgs::PointCloud2::ConstPtr l);
    void TopLidarDataTran(const sensor_msgs::PointCloud2::ConstPtr l);

    rosbag::Bag bag_;
    double base_lat_deg_ = 31.206388889;
    double base_lon_deg_ = 121.689444445;
    int utm_zone_ = 51;
    double ts_begin_;
    double ts_end_;
    float lidar_hz_ = 0.1;
    std::shared_ptr<rosbag::View> view_ptr_;
    std::vector<std::string> topics_;
    std::map<double, Eigen::Matrix4d> gps_msg_;
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> lidar_msg_;
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> toplidar_msg_;
    rosbag::View::iterator it_;
    std::string bag_path_;
    std::string pose_path_;
    std::unordered_map<std::string, std::vector<rosbag::MessageInstance>> msgs_;
};