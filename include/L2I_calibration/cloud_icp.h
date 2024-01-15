#pragma once

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include "common/bag_reader.h"
// #include <open3d/Open3D.h>

inline bool cloud_icp(std::string source_pcd, std::string target_pcd, std::string out_put_path,
                    Eigen::Matrix4d T_init, std::vector<Eigen::Matrix4d>& l2i_vec){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // Load two pcd files from disk
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_pcd, *cloud_in) == -1 
        || pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd, *cloud_out) == -1)
    {
        PCL_ERROR ("Couldn't read file \n");
        return false;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    int iterations =  5;
    icp.setMaximumIterations(iterations);
    // icp.setRotationThreshold(0.02);
    // icp.setTranslationThreshold(0.1);
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    Eigen::Matrix4f T_l2i_init = T_init.inverse().cast<float>();
    // std::cout << T_l2i_init << std::endl;

    for (size_t i = 0; i < cloud_out -> points.size(); i++)
    {
        cloud_out -> points[i].x -= T_l2i_init(0,3);
        cloud_out -> points[i].y -= T_l2i_init(1,3);
    }
    pcl::io::savePCDFileBinary (out_put_path+ "_tran_map.pcd", *cloud_out);
    T_l2i_init(0,3) = 0;
    T_l2i_init(1,3) = 0;
    icp.align(Final, T_l2i_init);
    std::cout << "ICP has converged: " << icp.hasConverged() << " score: " <<  icp.getFitnessScore() << std::endl;
    Eigen::Matrix4d T_l2w = (icp.getFinalTransformation()).cast<double>();
    Eigen::Matrix4d T_l2i = T_l2i_init.inverse().cast<double>() * T_l2w;
    std::cout << "icp Translation: " << T_l2i << std::endl;
    l2i_vec.push_back(T_l2i);
    pcl::io::savePCDFileBinary (out_put_path + "_align.pcd", Final);
    return true;
}

inline bool cloud_ndt(std::string source_pcd, std::string target_pcd, std::string out_put_path,
                    Eigen::Matrix4d T_init, std::vector<Eigen::Matrix4d>& l2i_vec) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // Load two pcd files from disk
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_pcd, *cloud_in) == -1 
        || pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd, *cloud_out) == -1)
    {
        PCL_ERROR ("Couldn't read file \n");
        return false;
    }
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon (0.01);// 为终止条件设置最小转换差异
    ndt.setStepSize (0.5);              // 为more-thuente线搜索设置最大步长
    ndt.setResolution (1.0);
    ndt.setMaximumIterations (10);

    ndt.setInputSource (cloud_in);  //源点云
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    Eigen::Matrix4f T ;
    T << 9.99995156e-01, -2.73800520e-03 ,-1.48054897e-03, -1.13426237e-02,
        2.73707990e-03,  9.99996058e-01, -6.26635115e-04 , 3.92399120e-01,
        1.48225886e-03 , 6.22579698e-04 , 9.99998708e-01,  1.14772168e-01,
        0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00;
    Eigen::Matrix4f T_l2i_init = T_init.inverse().cast<float>() * T;
    // std::cout << T_l2i_init << std::endl;
    ndt.align(Final, T_l2i_init);
    std::cout << "NDT has converged: " << ndt.hasConverged() << " score: " <<ndt.getFitnessScore() << std::endl;
    Eigen::Matrix4d T_l2w = (ndt.getFinalTransformation()).cast<double>();
    Eigen::Matrix4d T_l2i = T_init * T_l2w;
    std::cout << "NDT Translation: " << T_l2i << std::endl;
    l2i_vec.push_back(T_l2i);
    pcl::io::savePCDFileBinary(out_put_path, Final);
}



// bool cloud_icp_o3d(std::string source_pcd, std::string target_pcd,std::string out_put_path,
//                     Eigen::Matrix4d T_init, std::vector<Eigen::Matrix4d>& l2i_vec){
//     auto source = open3d::io::CreatePointCloudFromFile(source_pcd);
//     auto target = open3d::io::CreatePointCloudFromFile(target_pcd);
//     double threshold = 0.02;  
//     Eigen::Matrix4d T_l2i_init = T_init.inverse();
//     auto reg_p2p = open3d::pipelines::registration::RegistrationICP(
//         *source, *target, threshold,
//         T_l2i_init,
//         open3d::pipelines::registration::TransformationEstimationPointToPoint());
//     Eigen::Matrix4d T_l2w = reg_p2p.transformation_;
//     Eigen::Matrix4d T_l2i = T_init * T_l2w;
//     l2i_vec.push_back(T_l2i);
//     source->Transform(reg_p2p.transformation_);
//     open3d::io::WritePointCloud(out_put_path, *source);
//     std::cout << "Final transformation is:\n"
//               << T_l2i << std::endl;
//     std::cout << "Registration fitness: " << reg_p2p.fitness_ << std::endl;

// }
