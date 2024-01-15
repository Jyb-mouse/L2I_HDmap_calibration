#include "L2I_calibration/calibration.h"
#include "utils/filter_data.h"
#include "L2I_calibration/cloud_icp.h"
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "L2I_calibration/ndt.h"

L2I_Calibration::L2I_Calibration(const Config& cfg) : cfg_(cfg){

}

bool L2I_Calibration::Init(){
    int grid_size = cfg_.getvalue<int>("grid_size");
    int seg_num = cfg_.getvalue<int>("seg_num");
    std::string map_path = cfg_.getvalue<std::string>("map_path");
    std::string las_path = cfg_.getvalue<std::string>("las_path");
    std::string pose_path = cfg_.getvalue<std::string>("pose_path");
    pcd_grid_.Init(las_path , grid_size, seg_num);
    map_path_ = map_path;
    pose_path_ = pose_path;
    map_las_path_ = cfg_.getvalue<std::string>("map_las_path");
    std::string output_path = cfg_.getvalue<std::string>("output_path");
    output_path_ = output_path;
    std::string bag_path = cfg_.getvalue<std::string>("bag_path");
    double ts_begin = cfg_.getvalue<double>("ts_begin");
    double ts_end = cfg_.getvalue<double>("ts_end");
    filter_lidar_pts_threshold_x_ = cfg_.getvalue<double>("filter_lidar_pts_threshold_x");
    filter_lidar_pts_threshold_y_ = cfg_.getvalue<double>("filter_lidar_pts_threshold_y");
    filter_lidar_pts_threshold_z_ = cfg_.getvalue<double>("filter_lidar_pts_threshold_z");
    down_samples_ = cfg_.getvalue<int>("down_samples");
    std::vector<std::string> topics;
    std::string toplidar_topic = cfg_.getvalue<std::string>("toplidar_topic");
    std::string rearlidar_topic = cfg_.getvalue<std::string>("rearlidar_topic");
    std::string gps_topic = cfg_.getvalue<std::string>("gps_topic");
    topics.emplace_back(toplidar_topic);
    // topics.emplace_back(rearlidar_topic);
    topics.emplace_back(gps_topic);
    auto start_parse_bag = std::chrono::high_resolution_clock::now();
    bag_reader_.Init(bag_path, ts_begin, ts_end, topics, pose_path);
    bag_reader_.DataParse();
    auto end_parse_bag = std::chrono::high_resolution_clock::now();
    auto duration_parse_bag = std::chrono::duration_cast<std::chrono::microseconds>(end_parse_bag - start_parse_bag);
    // std::cout << "time of parse bag: " << duration_parse_bag.count() / 1000 << " ms." << std::endl;
    lidar_msg_ = bag_reader_.get_toplidar_msg();
    gps_msg_ = bag_reader_.get_gps_msg();
    ///TODO: false
    
    return true;
}

Eigen::Matrix4d L2I_Calibration::getCurTimePose(double cur_time){
    std::pair<double, Eigen::Matrix4d> next_nearest_frame = bag_reader_.GetNearestTimePose(cur_time);
    std::pair<double, Eigen::Matrix4d> last_nearest_frame = bag_reader_.GetNearestTimePose(cur_time - 0.015);
    Eigen::Matrix4d cur_pose = interpolate_pose(cur_time,   
                last_nearest_frame.first, last_nearest_frame.second,
                next_nearest_frame.first, next_nearest_frame.second);
    return cur_pose;
}

std::map<double, Eigen::Matrix4d> L2I_Calibration::Run(){
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr>::iterator it = lidar_msg_.begin();
    int frame = 1;
    std::map<double, Eigen::Matrix4d> last_res;
    for (; it != lidar_msg_.end(); it++)
    {
        if (frame % down_samples_ != 0)
        {
            frame++;
            continue;
        }
        // std::cout << "---------frame---------: " << frame << std::endl;
        auto tamp = it;
        --tamp;
        double cur_time = it -> first;
        double last_time = tamp -> first;
        pcl::PointCloud<RsPointXYZIRT_f>::Ptr cur_cloud = it -> second;
        Eigen::Matrix4d cur_lidar_time_pose = getCurTimePose(cur_time);
        Eigen::Matrix4d last_lidar_time_pose = getCurTimePose(last_time);
        Eigen::Matrix4d T_cur_2_last = last_lidar_time_pose.inverse() * cur_lidar_time_pose;
        std::string cur_lidar_dis_path = output_path_ + std::to_string(cur_time)+ "_dis_lidar";
        // pcl::io::savePCDFileBinary(cur_lidar_dis_path + ".pcd", *cur_cloud);
        RemoveLidarDistortion(cur_cloud, T_cur_2_last, last_time, cur_time);
        Eigen::Vector3d cur_xyz = cur_lidar_time_pose.block<3,1>(0,3);
        pcl::PointCloud<RsPointXYZIRT_f>::Ptr enu_local_point(new pcl::PointCloud<RsPointXYZIRT_f>());
        pcd_grid_.GetLocalMap(map_path_, cur_xyz, enu_local_point);

        Eigen::Matrix4d cur_Tw2i = cur_lidar_time_pose.inverse();
        std::string cur_submap_path;
        std::string cur_lidar_path;
        cur_submap_path = output_path_ + std::to_string(cur_time)+ "_submap_imu_cor.pcd";
        cur_lidar_path = output_path_ + std::to_string(cur_time)+ "_lidar.pcd";
        pcl::io::savePCDFileBinary(cur_submap_path, *enu_local_point);
        pcl::io::savePCDFileBinary(cur_lidar_path, *cur_cloud);
        last_res.insert({cur_time, cur_lidar_time_pose});
        frame++; 
        // bool icp_status = false;
        // std::string align_lidar_path;
        // align_lidar_path = output_path_ + std::to_string(cur_time);
        // icp_status = cloud_ndt(cur_lidar_path, cur_submap_path, align_lidar_path, cur_Tw2i, l2i_vec_);
    }   

    return last_res;
}

PointCloud::Ptr L2I_Calibration::FilterLidar(pcl::PointCloud<RsPointXYZIRT_f>::Ptr &cloud,
                        pcl::PointCloud<RsPointXYZIRT_f>::Ptr &pc_in,
                        Eigen::Matrix4d T, std::string path){
    // 创建 PassThrough 过滤器
    PointCloud::Ptr lidar_points(new PointCloud);
    for(int i = 0; i < cloud -> points.size(); i++){
        if(cloud->points[i].z < -filter_lidar_pts_threshold_z_ && 
           cloud->points[i].y < filter_lidar_pts_threshold_y_ &&
           cloud->points[i].y > -filter_lidar_pts_threshold_y_ && 
           cloud->points[i].x < filter_lidar_pts_threshold_x_ &&
           cloud->points[i].x > -filter_lidar_pts_threshold_x_){
                Point3d point;
                point[0] = cloud->points[i].x;
                point[1] = cloud->points[i].y;
                point[2] = cloud->points[i].z;
                point.intensity=0;
                point.ring = 0;
                point.timestamp=0;
                if (!std::isfinite (point[0]) || 
                                    !std::isfinite (point[1]) || 
                                    !std::isfinite (point[2]))
                                    continue;
                if (std::isnan (point[0]) || 
                                    std::isnan (point[1]) || 
                                    std::isnan (point[2]))
                                    continue;

                lidar_points->points.push_back(point);
            }
    }


    pc_in -> width = lidar_points->points.size();                                 //设置点云宽度
    pc_in -> height = 1;                                //设置点云高度
    pc_in -> is_dense = false;                          //非密集型
    pc_in -> points.resize(pc_in ->width * pc_in ->height); 

    for (size_t i = 0; i < pc_in ->points.size(); ++i)
    { 
      Eigen::Vector4d p;
      p << lidar_points->points[i][0], lidar_points->points[i][1], lidar_points->points[i][2], 1;
    //   Eigen::Vector4d p_world = T * p;
      pc_in ->points[i].x = p(0);
      pc_in ->points[i].y = p(1);
      pc_in ->points[i].z = p(2);
      pc_in ->points[i].intensity = 0;
      pc_in ->points[i].ring = 0;
      pc_in ->points[i].timestamp = 0;
    }

    // 输出处理后的点云
    pcl::io::savePCDFile<RsPointXYZIRT_f>(path, *pc_in);
    return lidar_points;
} 

PointCloud::Ptr Read_las(std::string las_path, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in){
    PointCloud::Ptr lidar_points(new PointCloud);
    LASreadOpener lasrReadOpener;
    LASreader *lasReader = lasrReadOpener.open(las_path.c_str(), false);
    if (lasReader != NULL) {
        size_t count = lasReader->header.number_of_point_records;
        lidar_points->reserve(count);
        for (size_t j = 0; lasReader->read_point() && j < count; ++j) {
            Point3d point;
            double z = lasReader->point.get_z();
            if(lasReader->point.get_z() < 18.0){
                point[0] = lasReader->point.get_x() - 374950;
                point[1] = lasReader->point.get_y() - 3447440;
                point[2] = lasReader->point.get_z();
                point.intensity=lasReader->point.get_intensity();
                point.ring = 0;
                point.timestamp=lasReader->point.get_gps_time();
                if (!std::isfinite (point[0]) || 
                                    !std::isfinite (point[1]) || 
                                    !std::isfinite (point[2]))
                                    continue;
                if (std::isnan (point[0]) || 
                                    std::isnan (point[1]) || 
                                    std::isnan (point[2]))
                                    continue;

                lidar_points->points.push_back(point);
            }
        }
        lasReader->close();
    } else {
        spdlog::info("read las file_path fail! {}", las_path);
        lasReader->close();
    }
    pc_in -> width = lidar_points->points.size();                                 //设置点云宽度
    pc_in -> height = 1;                                //设置点云高度
    pc_in -> is_dense = false;                          //非密集型
    pc_in -> points.resize(pc_in ->width * pc_in ->height); 

    for (size_t i = 0; i < pc_in ->points.size(); ++i)
    { 
      pc_in ->points[i].x = lidar_points->points[i][0];
      pc_in ->points[i].y = lidar_points->points[i][1];
      pc_in ->points[i].z = lidar_points->points[i][2];
      pc_in ->points[i].intensity = 0;
    //   pc_in ->points[i].ring = 0;
    //   pc_in ->points[i].timestamp = 0;
    }
    return lidar_points;
}

inline Eigen::Vector3d ENURotToInsRPY(const Eigen::Matrix3d& rot_body_enu) {
  double pitch = asin(rot_body_enu(2, 1));
  double roll = atan2(-rot_body_enu(2, 0), rot_body_enu(2, 2));
  double yaw = atan2(-rot_body_enu(0, 1), rot_body_enu(1, 1));
  return Eigen::Vector3d{roll, pitch, yaw};
}

std::map<double, Eigen::Matrix4d> L2I_Calibration::Run2(){
    std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr>::iterator it = lidar_msg_.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloud::Ptr source_me(new PointCloud);
    PointCloud::Ptr target_me(new PointCloud);

    target_me = Read_las(map_las_path_, target);
    bool success;
    Ndt::Options options;
    options.voxel_size_ = 0.1;
    options.remove_centroid_ = false;
    options.nearby_type_ = Ndt::NearbyType::NEARBY6;
    Ndt ndt(options);
    ndt.SetTarget(target_me);
    std::map<double, Eigen::Matrix4d> last_res;
    for (; it != lidar_msg_.end(); it++)
    {

        auto tamp = it;
        --tamp;
        double cur_time = it -> first;
        double last_time = tamp -> first;
        pcl::PointCloud<RsPointXYZIRT_f>::Ptr cur_cloud = it -> second;
        Eigen::Matrix4d cur_lidar_time_pose = getCurTimePose(cur_time);
        Eigen::Matrix4d last_lidar_time_pose = getCurTimePose(last_time);
        Eigen::Matrix4d T_cur_2_last = last_lidar_time_pose.inverse() * cur_lidar_time_pose;
        pcl::PointCloud<RsPointXYZIRT_f>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT_f>());
        RemoveLidarDistortion(cur_cloud, T_cur_2_last, last_time, cur_time);
        // FilterLidar(cur_cloud, pc_in);
        Eigen::Matrix4d Tl2i;

        Tl2i << 0.9996539041998037,0.02467177360400634,-0.009131013378722592,-0.02599951,
            -0.02466335352205607,0.9996952788887938, 0.001033615258626519, 0.21332493,
            0.009153732087833868,-0.0008080561177524306, 0.9999577772257061, 0.37190165,
            0.0 ,0.0, 0.0,1.0;
        Eigen::Matrix4d Tl2w = cur_lidar_time_pose * Tl2i;
        std::string cur_lidar_dis_path = output_path_ + std::to_string(cur_time)+ "_lidar_00.pcd";
        source_me = FilterLidar(cur_cloud, pc_in, Tl2w, cur_lidar_dis_path);
        // std::cout << Tl2w << std::endl;
        Tl2w(0,3) -= 374950;
        Tl2w(1,3) -= 3447440;

        // std::string source_path =  "/media/data/data/L2I/GS4-002/vslam/5.pcd";
        // pcl::io::savePCDFileBinary(path, *target);
        // pcl::io::loadPCDFile(cur_lidar_dis_path, *source);
        
        // for (int i = 0; i < source -> points.size(); i++)
        // {
        //     Point3d p(source -> points[i].x, source -> points[i].y, source -> points[i].z,0,0,0);
        //     source_me -> points.push_back(p);
        // }
        Eigen::Matrix4d init_pose = Tl2w;
        ndt.SetSource(source_me);
        success = ndt.AlignNdt(Tl2w);
        // if (success)
        // {
        //     std::cout << "Delta_T:" << init_pose.inverse() * Tl2w << std::endl;
        // }
        Eigen::Matrix4d delta_T = init_pose.inverse() * Tl2w;
        Eigen::Matrix3d new_R = delta_T.block<3,3>(0,0);
        Eigen::Vector3d eulerAngle=ENURotToInsRPY(new_R) * (180 / M_PI);
        std::cout << eulerAngle.transpose()(0) << " "
                  << eulerAngle.transpose()(1) << " "
                  << eulerAngle.transpose()(2) << " "
                  << delta_T(0, 3) << " "
                  << delta_T(1, 3) << " "
                  << delta_T(2, 3) << " "
                  << std::endl;
        // rotation_list.push_back(eulerAngle);
        // tran_list.push_back(delta_T.block<3,1>(0,3));
    }
    return last_res;
}