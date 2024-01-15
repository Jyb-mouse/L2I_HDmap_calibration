#pragma once

#include <vector>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include "common/point.h"

class Ndt
{
public:
   
    enum class NearbyType{
        CENTER,
        NEARBY6,
    };

    struct Options{
        int max_iteration_ = 50;
        double voxel_size_ = 1.0;
        double inv_voxel_size_ = 1.0;
        int min_effective_pts_ = 10;
        int min_pts_in_voxel_ = 3;
        double eps_ = 1e-2;
        double res_outlier_th_ = 40.0;
        bool remove_centroid_ = false;

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    using KeyType = Eigen::Matrix<int,3,1>;
    struct VoxelData
    {
        VoxelData() {}
        VoxelData(size_t id) {idx_.emplace_back(id);}
        std::vector<size_t> idx_;
        Eigen::Vector3d mu_ = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sigma_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d info_ = Eigen::Matrix3d::Zero();
    };

    Ndt() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }
    explicit Ndt(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    ~Ndt() = default;

    void SetTarget(PointCloud::Ptr target) {
        target_ = target;
        BuildVoxels();

        target_center_ = std::accumulate(target -> points.begin(), target -> points.end(), Eigen::Vector3d::Zero().eval(), 
                                [](const Eigen::Vector3d& c, const Point3d& pt) -> Eigen::Vector3d {return c + ToVec3d(pt); }) /
                                target_ -> size();
    }

    void SetSource(PointCloud::Ptr source) {
        source_ = source;

        source_center_ = std::accumulate(source -> points.begin(), source -> points.end(), Eigen::Vector3d::Zero().eval(), 
                                [](const Eigen::Vector3d& c, const Point3d& pt) -> Eigen::Vector3d {return c + ToVec3d(pt); }) /
                                source_ -> size();
    }

    void SetGtPose(const Eigen::Matrix4d& gt_pose){
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    bool AlignNdt(Eigen::Matrix4d& pose_init);

private:
    void BuildVoxels();

    void GenerateNearbyGrids();

    PointCloud::Ptr target_ = nullptr;
    PointCloud::Ptr source_ = nullptr;
    
    Eigen::Vector3d target_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center_ = Eigen::Vector3d::Zero();

    Eigen::Matrix4d gt_pose_;
    bool gt_set_ =false;

    Options options_;

    std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_;
    std::vector<KeyType> nearby_grids_;

};

