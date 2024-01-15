
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl_conversions/pcl_conversions.h>
#include "common/bag_reader.h"
#include "transform/enu_transform.h"
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <cmath>
#include "omp.h"

BagReader::BagReader(std::string bag_path, double ts_begin, double ts_end, std::vector<std::string> topics,std::string pose_path)
    : bag_path_(bag_path),
      ts_begin_(ts_begin),
      ts_end_(ts_end),
      topics_(topics),
      pose_path_(pose_path) {}

BagReader::~BagReader() {}

void BagReader::Init(std::string bag_path, double ts_begin, double ts_end, std::vector<std::string> topics, std::string pose_path){
    bag_path_ = bag_path;
    ts_begin_ = ts_begin;
    ts_end_ = ts_end;
    topics_ = topics;
    pose_path_ = pose_path;
}

void BagReader::DataParse() {
    bag_.open(bag_path_, rosbag::bagmode::Read);
    view_ptr_.reset(new rosbag::View());
    if (ts_begin_ > 0 && ts_end_ > 0)
    {
        view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics_), ros::Time(ts_begin_), ros::Time(ts_end_));
    } else if (ts_begin_ > 0)
    {
        view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics_), ros::Time(ts_begin_), ros::TIME_MAX);
    } else {
        view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics_));
    }
    it_ = view_ptr_->begin();
    EnuTransform enu_transform("EPSG:32651", base_lat_deg_, base_lon_deg_, utm_zone_);
    for (; it_ != view_ptr_ -> end(); ++it_)
    {
        auto m = *it_;
        std::string topic = m.getTopic();
        if (topic == "/rtkimu/hc/220pro/nav")
        {
            sensor_msgs::NavSatFix::ConstPtr gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
            GpsDataTran(gps_msg);
            msgs_[topic].emplace_back(m);
        } else if (topic == "/met/lidar")
        {
            sensor_msgs::PointCloud2::ConstPtr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            LidarDataTran(lidar_msg);
            msgs_[topic].emplace_back(m);
        } else if (topic == "/lidar/rs/rubyplus128/topmiddle")
        {
            sensor_msgs::PointCloud2::ConstPtr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            TopLidarDataTran(lidar_msg);
            msgs_[topic].emplace_back(m);
        }
    }
    // txt_to_queue(pose_path_);

}

void BagReader::GpsDataTran(const sensor_msgs::NavSatFix::ConstPtr p) {
    double timestamp = p -> header.stamp.toSec();
    double latitude = p -> latitude;
    double longitude = p -> longitude;
    double altitude = p -> altitude;

    EnuTransform enu_transform("EPSG:32651", base_lat_deg_, base_lon_deg_, utm_zone_);
    double yaw_bias = enu_transform.get_yaw_bias(latitude, longitude);
    double roll = (p -> position_covariance.elems[1]) * M_PI /180.0;
    double pitch = (p -> position_covariance.elems[2]) * M_PI /180.0;
    double yaw = ((-(p -> position_covariance.elems[3])) + yaw_bias) * M_PI /180.0;
    Eigen::Matrix3d R_imu2enu = enu_transform.rpy2rotation(roll, pitch,yaw);
    Eigen::Vector3d lla{latitude, longitude, altitude};
    Eigen::Vector3d xyz = enu_transform.LLA2XYZ(lla);
    Eigen::Matrix4d T_imu2enu;
    T_imu2enu.setIdentity();
    T_imu2enu.block<3,3>(0,0) = R_imu2enu;
    T_imu2enu.block<3,1>(0,3) = xyz;
    // std::cout << "timestamp: " << timestamp << std::endl;
    // std::cout << T_imu2enu<< std::endl;
    gps_msg_.insert({timestamp, T_imu2enu});
}
inline std::vector<std::string> split(std::string str, std::string pattern) {
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;  // 扩展字符串以方便操作
  int size = str.size();
  for (int i = 0; i < size; i++) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

void BagReader::txt_to_queue(std::string filename){
    std::map<double, Eigen::Matrix4d> poseMsgQueue;
    std::ifstream inFile;
    inFile.open(filename);
    if (!inFile) {
        SPDLOG_ERROR("Unable to open {}", filename);
        throw std::runtime_error(fmt::format("Unable to open {}", filename));
    }
    std::string s;
    while (getline(inFile, s)) {
        std::vector<std::string> elements;
        elements = split(s, ",");
        double time_r = std::stod(elements[0].c_str());

        Eigen::Quaterniond q =Eigen::Quaterniond::Identity();
        Eigen::Vector3d xyz(0,0,0);

        if (fabs(std::stod(elements[1].c_str())) > 1) {
        q.x() = std::stod(elements[4].c_str());
        q.y() = std::stod(elements[5].c_str());
        q.z() = std::stod(elements[6].c_str());
        q.w() = std::stod(elements[7].c_str());
        xyz(0) = std::stod(elements[1].c_str());
        xyz(1) = std::stod(elements[2].c_str());
        xyz(2) = std::stod(elements[3].c_str());
        } else {
        q.x() = std::stod(elements[1].c_str());
        q.y() = std::stod(elements[2].c_str());
        q.z() = std::stod(elements[3].c_str());
        q.w() = std::stod(elements[4].c_str());
        xyz(0) = std::stod(elements[5].c_str());
        xyz(1) = std::stod(elements[6].c_str());
        xyz(2) = std::stod(elements[7].c_str());
        }

        Eigen::Matrix3d rot = q.toRotationMatrix();
        Eigen::Matrix4d T_imu2enu;
        T_imu2enu.setIdentity();
        T_imu2enu.block<3,3>(0,0) = rot;
        T_imu2enu.block<3,1>(0,3) = xyz;

        gps_msg_.insert({time_r, T_imu2enu});

    }
}


void BagReader::LidarDataTran(const sensor_msgs::PointCloud2::ConstPtr l){

    double timestamp = l -> header.stamp.toSec();
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr laserCloudFullRes(new pcl::PointCloud<RsPointXYZIRT_f>());
    pcl::fromROSMsg(*l, *laserCloudFullRes);
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr pcd_cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
    for (size_t i = 0; i < laserCloudFullRes -> points.size(); i++)
    {
        if (!std::isfinite (laserCloudFullRes -> points[i].x) || 
            !std::isfinite (laserCloudFullRes -> points[i].y) || 
            !std::isfinite (laserCloudFullRes -> points[i].z))
            continue;
        if (std::isnan (laserCloudFullRes -> points[i].x) || 
            std::isnan (laserCloudFullRes -> points[i].y) || 
            std::isnan (laserCloudFullRes -> points[i].z))
            continue;
        RsPointXYZIRT_f point;
        point.x = laserCloudFullRes -> points[i].x;
        point.y = laserCloudFullRes -> points[i].y;
        point.z = laserCloudFullRes -> points[i].z;
        point.intensity = laserCloudFullRes -> points[i].intensity;
        point.ring = laserCloudFullRes -> points[i].ring;
        point.timestamp = laserCloudFullRes -> points[i].timestamp;
        pcd_cloud -> points.push_back(point);
    }
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
    cloud->width = pcd_cloud->points.size();                                 //设置点云宽度
    cloud->height = 1;                                //设置点云高度
    cloud->is_dense = false;                          //非密集型
    cloud->points.resize(cloud->width * cloud->height); 

    for (size_t i = 0; i < cloud->points.size(); ++i)
    { 
      cloud->points[i].x = pcd_cloud->points[i].x;
      cloud->points[i].y =pcd_cloud->points[i].y;
      cloud->points[i].z = pcd_cloud->points[i].z;
      cloud->points[i].intensity = pcd_cloud->points[i].intensity;
      cloud->points[i].ring = pcd_cloud->points[i].ring;
      cloud->points[i].timestamp = pcd_cloud->points[i].timestamp;
    }
    lidar_msg_.insert({timestamp, cloud});
}

void BagReader::TopLidarDataTran(const sensor_msgs::PointCloud2::ConstPtr l){

    double timestamp = l -> header.stamp.toSec();
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr laserCloudFullRes(new pcl::PointCloud<RsPointXYZIRT_f>());
    pcl::fromROSMsg(*l, *laserCloudFullRes);
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr pcd_cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
// #pragma omp parallel for num_threads(10)
    for (size_t i = 0; i < laserCloudFullRes -> points.size(); i++)
    {
        if (!std::isfinite (laserCloudFullRes -> points[i].x) || 
            !std::isfinite (laserCloudFullRes -> points[i].y) || 
            !std::isfinite (laserCloudFullRes -> points[i].z))
            continue;
        if (std::isnan (laserCloudFullRes -> points[i].x) || 
            std::isnan (laserCloudFullRes -> points[i].y) || 
            std::isnan (laserCloudFullRes -> points[i].z))
            continue;
        RsPointXYZIRT_f point;
        point.x = laserCloudFullRes -> points[i].x;
        point.y = laserCloudFullRes -> points[i].y;
        point.z = laserCloudFullRes -> points[i].z;
        point.intensity = laserCloudFullRes -> points[i].intensity;
        point.ring = laserCloudFullRes -> points[i].ring;
        point.timestamp = laserCloudFullRes -> points[i].timestamp;
        pcd_cloud -> points.push_back(point);
    }
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
    cloud->width = pcd_cloud->points.size();                                 //设置点云宽度
    cloud->height = 1;                                //设置点云高度
    cloud->is_dense = false;                          //非密集型
    cloud->points.resize(cloud->width * cloud->height); 
// #pragma omp parallel for num_threads(10)

    for (size_t i = 0; i < cloud->points.size(); ++i)
    { 
      cloud->points[i].x = pcd_cloud->points[i].x;
      cloud->points[i].y =pcd_cloud->points[i].y;
      cloud->points[i].z = pcd_cloud->points[i].z;
      cloud->points[i].intensity = pcd_cloud->points[i].intensity;
      cloud->points[i].ring = pcd_cloud->points[i].ring;
      cloud->points[i].timestamp = pcd_cloud->points[i].timestamp;
    }
    toplidar_msg_.insert({timestamp, cloud});
    // std::string file_path = "/media/data/output/GS4-001_las/2023-08-08-23-20-35/toplidar/" + std::to_string(timestamp) + ".pcd";
    // pcl::io::savePCDFileBinary(file_path, *cloud);
}

std::pair<double, Eigen::Matrix4d> BagReader::GetNearestTimePose(double t){
    auto pose = gps_msg_.lower_bound(t);
    return {pose->first,pose -> second};
}
// bool BagReader::Reset(double ts_begin, double ts_end, std::vector<std::string> hard_topics,
//                       std::vector<std::string> soft_topics){
//     ts_begin_ = ts_begin;
//     ts_end_ = ts_end;
//     return Reset(hard_topics, soft_topics);
// }

// bool BagReader::Reset(std::vector<std::string> hard_topics, std::vector<std::string> soft_topics){
//     hard_topics_ = hard_topics;
//     soft_topics_ = soft_topics;
//     aligner_.Reset(lidar_hz_, hard_topics, soft_topics);
//     if (!bag_.isOpen())
//     {
//         SPDLOG_INFO("open bag {}", bag_path_);
//         bag_.open(bag_path_, rosbag::bagmode::Read);
//         SPDLOG_INFO("reading bag {}... ts_begin:{} ts_end:{}", bag_path_, ts_begin_, ts_end_);
//     }
//     std::vector<std::string>  topics = hard_topics;
//     topics.insert(topics.end(), soft_topics.begin(), soft_topics.end());
//     view_ptr_.reset(new rosbag::View());
//     if (ts_begin_ > 0 && ts_end_ > 0)
//     {
//         view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics), ros::Time(ts_begin_), ros::Time(ts_end_));
//     } else if(ts_begin_ > 0) {
//         view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics), ros::Time(ts_begin_), ros::TIME_MAX);
//     } else {
//         view_ptr_ -> addQuery(bag_, rosbag::TopicQuery(topics));
//     }
    
//     it_ = view_ptr_ -> begin();

//     return true;
// }

// bool BagReader::Next() {
//     if (!bag_.isOpen())
//     {
//         Reset(ts_begin_, ts_end_, hard_topics_, soft_topics_);
//     }
//     for (; it_ != view_ptr_-> end(); ++it_)
//     {
//         if (aligner_.PushData(*it_))
//         {
//             return true;
//         }
//     }
//     return false;
// }
