#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <dirent.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unordered_map>
#include <unordered_set>
#include <map>
#include "lasreader.hpp"
#include "laswriter.hpp"
#include "common/bag_reader.h"
#include "omp.h"

inline Eigen::Matrix4d interpolate_pose(double cur_ts, 
        double last_ts, Eigen::Matrix4d last_pose,
        double next_ts, Eigen::Matrix4d next_pose){
    if (next_ts == last_ts)
    {
      return next_pose;
    }
    double w1 = (cur_ts - last_ts) / (next_ts - last_ts);
    double w2 = (next_ts - cur_ts) / (next_ts - last_ts);
    Eigen::Matrix3d last_pose_rotation = last_pose.block<3,3>(0,0);
    Eigen::Vector3d last_pose_trans = last_pose.block<3,1>(0,3);
    Eigen::Matrix3d next_pose_rotation = next_pose.block<3,3>(0,0);
    Eigen::Vector3d next_pose_trans = next_pose.block<3,1>(0,3);
    Eigen::Quaterniond last_quaternion(last_pose_rotation);
    Eigen::Quaterniond next_quaternion(next_pose_rotation);
    Eigen::Quaterniond q = last_quaternion.slerp(w1, next_quaternion);
    Eigen::Vector3d t = w2 * last_pose_trans + w1 * next_pose_trans;
    Eigen::Matrix4d new_T = Eigen::Matrix4d::Identity();
    new_T.block<3,3>(0,0) = q.matrix();
    new_T.block<3,1>(0,3) = t;
    return new_T;
}

inline void getFilesList( std::vector<std::string>& file_path_full, std::vector<std::string>& file_name,
                  std::string dirpath, std::string suffix, bool recursive) {
  file_path_full.clear();
  file_name.clear();
  DIR* dir = opendir(dirpath.c_str());
  if (dir == NULL) {
    return;
  }
  std::vector<std::string> allPath;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type == DT_DIR) {  // It's dir
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
      std::string dirNew = dirpath + "/" + entry->d_name;
      std::vector<std::string> temp_path;
      std::vector<std::string> temp_filename;
      getFilesList(temp_path, temp_filename, dirNew, suffix, recursive);
      file_path_full.insert(file_path_full.end(), temp_path.begin(), temp_path.end());
      file_name.insert(file_name.end(), temp_filename.begin(), temp_filename.end());

    } else {
      // cout << "name = " << entry->d_name << ", len = " << entry->d_reclen << ", entry->d_type = "
      // << (int)entry->d_type << endl;
      std::string name = entry->d_name;
      if (name.find(suffix) != name.size() - suffix.size()) continue;
      std::string full_path = dirpath + "/" + name;
      // sprintf("%s",imgdir.c_str());
      file_path_full.push_back(full_path);
      file_name.push_back(name);
    }
  }
  closedir(dir);
}

inline void getFilesMap(std::map<double, std::string>& file_path_full,
                  std::string dirpath, std::string suffix, bool recursive) {
  file_path_full.clear();
  DIR* dir = opendir(dirpath.c_str());
  if (dir == NULL) {
    return;
  }
  std::vector<std::string> allPath;
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type == DT_DIR) {  // It's dir
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
      std::string dirNew = dirpath + "/" + entry->d_name;
      std::map<double, std::string> temp_path;
      getFilesMap(temp_path, dirNew, suffix, recursive);
      file_path_full.insert(temp_path.begin(), temp_path.end());

    } else {
      // cout << "name = " << entry->d_name << ", len = " << entry->d_reclen << ", entry->d_type = "
      // << (int)entry->d_type << endl;
      std::string name = entry->d_name;
      int len_sub_str = name.size() - suffix.size();
      std::string y_pose = name.substr(0, len_sub_str);
      double y_pose_double = stod(y_pose);
      if (name.find(suffix) != name.size() - suffix.size()) continue;
      std::string full_path = dirpath + "/" + name;
      // sprintf("%s",imgdir.c_str());
      file_path_full.insert({y_pose_double, full_path});
    }
  }
  closedir(dir);
}


inline void RemoveLidarDistortion(pcl::PointCloud<RsPointXYZIRT_f>::Ptr &cloud, 
            const Eigen::Matrix4d T_end2start, double time_start, double time_cur_lidar){
    int point_num = cloud -> points.size();
    double time_interval = time_cur_lidar - time_start;
    Eigen::Matrix3d last_next_rotation = T_end2start.block<3,3>(0,0);
    Eigen::Quaterniond last_quaternion(last_next_rotation);
    Eigen::Vector3d last_next_trans = T_end2start.block<3,1>(0,3);
#pragma omp parallel for num_threads(12)
    for (size_t i = 0; i < point_num; i++)
    {
        if (std::isnan(cloud ->points[i].x) || std::isnan(cloud ->points[i].y) || std::isnan(cloud ->points[i].z)) {
            continue;
        }
        if (cloud -> points[i].timestamp < time_start || cloud -> points[i].timestamp > time_cur_lidar)
        {
            continue;
        }
        Eigen::Vector3d start_point;

        float scale = (cloud -> points[i].timestamp - time_start) / time_interval;
        Eigen::Quaterniond delta_q = 
            (Eigen::Quaterniond::Identity().slerp(scale, last_quaternion)).normalized();
        Eigen::Vector3d delta_t = scale * last_next_trans;
        start_point = delta_q * Eigen::Vector3d(cloud -> points[i].x, cloud -> points[i].y, cloud -> points[i].z)
                        + delta_t;
        Eigen::Vector4d start_point_qi(start_point[0], start_point[1], start_point[2],1);
        Eigen::Vector4d end_point = T_end2start.inverse() * start_point_qi;
        cloud->points[i].x = end_point(0);
        cloud->points[i].y = end_point(1);
        cloud->points[i].z = end_point(2);
        cloud->points[i].timestamp = time_cur_lidar;
    }
}

inline void save_lidar_msg_to_pcd(const double timestamp, pcl::PointCloud<RsPointXYZIRT_f> cloud,
                           const std::string save_path){
    std::string file_path = save_path + "/" + std::to_string(timestamp) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, cloud);
}

inline void save_las2pcd(const std::string las_path, const std::string pcd_path) 
{
    std::vector<std::string> filenamelist;
    std::vector<std::string> file_name;
    getFilesList(filenamelist, file_name, las_path, ".las", true);
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr pcd_cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
    for(auto f_name:filenamelist){
        // pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        LASreadOpener lasrReadOpener;
        LASreader *lasReader = lasrReadOpener.open(f_name.c_str(), false);
        if (lasReader != NULL) {
            size_t count = lasReader->header.number_of_point_records;
            // pcd_cloud->points.reserve(count);
            for (size_t j = 0; lasReader->read_point() && j < count; ++j) {
                RsPointXYZIRT_f point;
                point.x = lasReader->point.get_x();
                point.y = lasReader->point.get_y();
                point.z = lasReader->point.get_z();
                point.intensity=lasReader->point.get_intensity();
                point.ring=0;
                point.timestamp=lasReader->point.get_gps_time();
                if (!std::isfinite (point.x) || 
                                    !std::isfinite (point.y) || 
                                    !std::isfinite (point.z))
                                    continue;
                if (std::isnan (point.x) || 
                                    std::isnan (point.y) || 
                                    std::isnan (point.z))
                                    continue;

                pcd_cloud->points.push_back(point);
            }
            lasReader->close();
        } else {
            spdlog::info("read las file_path fail! {}", f_name);
            lasReader->close();
        }
    }
    pcl::PointCloud<RsPointXYZIRT_f> cloud;
    cloud.width = pcd_cloud->points.size();                                 //设置点云宽度
    cloud.height = 1;                                //设置点云高度
    cloud.is_dense = false;                          //非密集型
    cloud.points.resize(cloud.width * cloud.height); 
    for (size_t i = 0; i < cloud.points.size(); ++i)
    { 
        // spdlog::info("point:{}",cloud.points[i].x);
        // spdlog::info("point:{}",pcd_cloud->points[i].x);
      cloud.points[i].x = pcd_cloud->points[i].x;
      cloud.points[i].y =pcd_cloud->points[i].y;
      cloud.points[i].z = pcd_cloud->points[i].z;
      cloud.points[i].intensity = pcd_cloud->points[i].intensity;
      cloud.points[i].ring = pcd_cloud->points[i].ring;
      cloud.points[i].timestamp = pcd_cloud->points[i].timestamp;
    }
    pcl::io::savePCDFileBinary (pcd_path, cloud);
}