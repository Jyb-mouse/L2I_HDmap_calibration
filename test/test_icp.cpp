#include <string>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Geometry>

#include "L2I_calibration/icp.h"
#include "L2I_calibration/ndt.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include </usr/local/include/liblas/liblas.hpp>
#include </usr/local/include/liblas/point.hpp>
#include </usr/local/include/liblas/version.hpp>
#include "utils/filter_data.h"
#include "common/bag_reader.h"
#include "L2I_calibration/cloud_icp.h"
#include "common/pcd_grid.h"
#include "common/config.h"
#include "L2I_calibration/calibration.h"
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

int main(int argc, char** argv){

    // ros::init (argc, argv, "bag_reader");
    // rosbag::Bag bag;

    // std::string config_path = "/media/data/L2I_calibration/config/config.cfg";
    // Config cfg(config_path);

    // // L2I_Calibration::Ptr l2i_calibration(new L2I_Calibration(cfg));
    // // l2i_calibration -> Init();
    // // l2i_calibration -> Run();
    // // std::string config_path = "/media/data/L2I_calibration/config/config.cfg";
    // // Config cfg(config_path);

    // int grid_size = cfg.getvalue<int>("grid_size");
    // std::string map_path = cfg.getvalue<std::string>("map_path");
    // std::string las1_path = cfg.getvalue<std::string>("las_path");
    // std::string output_path = cfg.getvalue<std::string>("output_path");
    // std::string bag_path = cfg.getvalue<std::string>("bag_path");
    // double ts_begin = cfg.getvalue<double>("ts_begin");
    // double ts_end = cfg.getvalue<double>("ts_end");
    
    // // PCDGrid pcd_grid(las_path, grid_size);
    // // pcd_grid.LoadLasCloud(las_path, map_path);
    // std::vector<std::string> topics; 
    // topics.push_back(std::string("/rtkimu/hc/220pro/nav"));         
    // // topics.push_back(std::string("/met/lidar"));      
    // topics.push_back(std::string("/lidar/rs/rubyplus128/topmiddle"));      
    // BagReader bag_reader(bag_path, ts_begin, ts_end, topics, bag_path);
    // bag_reader.DataParse();
    // std::map<double, pcl::PointCloud<RsPointXYZIRT_f>::Ptr> toplidar_msg = bag_reader.get_toplidar_msg();
    // std::map<double, Eigen::Matrix4d> gps_msg = bag_reader.get_gps_msg();



    std::string las_path = "/media/data/data/L2I/GS4-002/vslam";
    std::string target_path = "/mnt/share/jinyanbin/geojson/las/2023-08-08-23-20-35/1691508141.200_tail.las";
    std::vector<std::string> filenamelist;
    std::vector<std::string> file_name;
    getFilesList(filenamelist, file_name, las_path, ".las", true);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloud::Ptr source_me(new PointCloud);
    PointCloud::Ptr target_me(new PointCloud);
    // pcl::PointCloud<PointXYZI>::Ptr point_in(new pcl::PointCloud<PointXYZI>());
    // source_me = Read_las(source_path);
    target_me = Read_las(target_path, target);
    std::string path = "/media/data/data/L2I/GS4-002/vslam/0.pcd";
    std::string source_path =  "/media/data/data/L2I/GS4-002/vslam/5.pcd";
    pcl::io::savePCDFileBinary(path, *target);
    pcl::io::loadPCDFile(source_path, *source);
    
    for (int i = 0; i < source -> points.size(); i++)
    {
        Point3d p(source -> points[i].x, source -> points[i].y, source -> points[i].z,0,0,0);
        source_me -> points.push_back(p);
    }

    std::vector<Eigen::Vector3d> tran_list;
    std::vector<Eigen::Vector3d> rotation_list;
    for (int i = 0; i < filenamelist.size(); i++)
    {
        std::string cur_las_path = filenamelist[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);

        // source_me = Read_las("/media/data/data/L2I/GS4-002/vslam/4.pcd", source);
        // std::string path = "/media/data/data/L2I/GS4-002/vslam/1.pcd";
        // pcl::io::savePCDFileBinary(path, *source);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::transformPointCloud(*source, *transformed_cloud, pose);
        // std::string path = "/media/data/data/L2I/GS4-002/vslam/0.pcd";
        // pcl::io::savePCDFileBinary(path, *source_me);
        Eigen::Matrix4d pose;
        pose <<   -0.998794,   0.0481068, -0.00980717,      -1.49339283,
                -0.0483149,   -0.998585,   0.0222201, -2.48030173,
                -0.00872435,   0.0226672,    0.999705,     19.092289599443028,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;
        Eigen::Matrix4d init_pose = pose;
        bool success;
        Ndt::Options options;
        options.voxel_size_ = 0.1;
        options.remove_centroid_ = false;
        options.nearby_type_ = Ndt::NearbyType::NEARBY6;
        Ndt ndt(options);
        ndt.SetSource(source_me);
        ndt.SetTarget(target_me);
        success = ndt.AlignNdt(pose);
        if (success)
        {
            std::cout << init_pose.inverse() * pose << std::endl;
        }
        Eigen::Matrix4d delta_T = init_pose.inverse() * pose;
        Eigen::Matrix3d new_R = delta_T.block<3,3>(0,0);
        Eigen::Vector3d eulerAngle=new_R.eulerAngles(2,1,0);
        std::cout << "eulerAngle, yaw, picth ,roll = " << eulerAngle << std::endl;
        rotation_list.push_back(eulerAngle);
        tran_list.push_back(delta_T.block<3,1>(0,3));
    }
    //mean
    double z_sum = 0.0;
    double roll_sum = 0.0;
    double pitch_sum = 0.0;
    int num = 0;
    for(int i = 0; i < tran_list.size(); i++){
        if(std::abs(rotation_list[i](0)) < 1.0){
            roll_sum += rotation_list[i](2);
            pitch_sum += rotation_list[i](1);
            z_sum += tran_list[i](2);
            num += 1;
        }
    }
    double z_mean = z_sum / num;
    double roll_mean = roll_sum / num;
    double pitch_mean = pitch_sum / num;

    std::cout << "z_mean = " << z_mean << std::endl;
    std::cout << "roll_mean = " << roll_mean << std::endl;
    std::cout << "pitch_mean = " << pitch_mean << std::endl;
    // pcl::io::loadPCDFile(source_path, *source);
    // pcl::io::loadPCDFile(target_path, *target);

    // for (int i = 0; i < source -> points.size(); i++)
    // {
    //     Point3d p(source -> points[i].x, source -> points[i].y, source -> points[i].z,0,0,0);
    //     source_me -> points.push_back(p);
    // }
    // for (int i = 0; i < target -> points.size(); i++)
    // {
    //     Point3d p(target -> points[i].x, target -> points[i].y, target -> points[i].z,0,0,0);
    //     target_me -> points.push_back(p);
    // }
    // Eigen::Matrix4d pose;
    // pose << 1, 0, 0, 0,
    //         0, 1, 0, -6.0,
    //         0, 0, 1, 0,
    //          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;

    // bool success;
    // Ndt::Options options;
    // options.voxel_size_ = 0.1;
    // options.remove_centroid_ = false;
    // options.nearby_type_ = Ndt::NearbyType::NEARBY6;
    // Ndt ndt(options);
    // ndt.SetSource(source_me);
    // ndt.SetTarget(target_me);
    // success = ndt.AlignNdt(pose);

    // ICP icp;
    // icp.SetSource(source_me);
    // icp.SetTarget(target_me);
    // // Eigen::Quaterniond a(1, 0, 0, 0);
    // // Eigen::Matrix3d R = a.toRotationMatrix(); 
    
    // // pose << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    // Eigen::Matrix4d init_pose;
    // init_pose << -9.97946171e-01,  5.67148436e-02, -2.97803040e-02,  3.75035117e+05,
    //             -5.72678197e-02, -9.98195571e-01,  1.80554132e-02,  3.44638258e+06,
    //             -2.87025576e-02,  1.97237836e-02,  9.99393384e-01,  1.91259837e+01,
    //              0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;
    // success = icp.AlignP2P(pose);
    // if (success)
    // {
    //     std::cout << pose << std::endl;
    // }
    // Eigen::Matrix3d new_R = pose.block<3,3>(0,0);
    // Eigen::Vector3d eulerAngle=new_R.eulerAngles(2,1,0);
    // std::cout << "eulerAngle, yaw, picth ,roll = " << eulerAngle << std::endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::transformPointCloud(*source, *transformed_cloud, pose);
    // std::string path = "/media/data/output/enu_map_220/0.pcd";
    // pcl::io::savePCDFileBinary(path, *transformed_cloud);
    return 0;
}