#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <numeric>
#include <glog/logging.h>
#include "common/point.h"
#include "L2I_calibration/kdtree.h"



using Mat6d = Eigen::Matrix<double, 6, 6>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec3d = Eigen::Vector3d;
class ICP
{   
  public:
    struct Options
    {
        int max_iteration_ = 50;
        double max_nn_distance_ = 1.0;
        double max_plane_distance_ = 0.05;
        double max_line_distance_ = 0.5;
        double eps_ = 1e-2;
        int min_effective_pts_ = 10;
        bool use_initial_translation_ = true;
    };

    ICP() {}
    explicit ICP(Options option): option_(option) {}
    ~ICP() = default;

    void SetTarget(PointCloud::Ptr target){
        // for (int i = 0; i < target -> points.size(); i++)
        // {
        //   target_ -> points.push_back(target -> points[i]);
        // }
        // target_ -> points = target -> points;
        target_ = target;
        BuildTargetKdTree();

        target_center_ = std::accumulate(target_ -> points.begin(), target_ -> points.end(), Eigen::Vector3d::Zero().eval(),
                    [](const Eigen::Vector3d& c, const Point3d& pt) -> Eigen::Vector3d { return c + ToVec3d(pt);}) / 
                    target_ -> size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }
    void SetSource(PointCloud::Ptr source){
        // for (int i = 0; i < source -> points.size(); i++)
        // {
        //   Point3d p = source -> points[i];
        //   source_ -> points.push_back(p);
        // }
        
        // source_ -> points = source -> points;
        source_= source;
        source_center_ = std::accumulate(source_ -> points.begin(), source -> points.end(), Eigen::Vector3d::Zero().eval(),
                    [](const Eigen::Vector3d& c, const Point3d& pt) -> Eigen::Vector3d { return c + ToVec3d(pt);}) / 
                    source -> size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    bool AlignP2P(Eigen::Matrix4d& init_pose);
    // bool AlignP2L(Eigen::Matrix4d& init_pose);
    // bool AlignP2Plane(Eigen::Matrix4d& init_pose);

  private:
    void BuildTargetKdTree();
    std::shared_ptr<KdTree> kdtree_ = nullptr;

    Options option_;
    PointCloud::Ptr target_;
    PointCloud::Ptr source_;

    Eigen::Vector3d target_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center_ = Eigen::Vector3d::Zero();
};

