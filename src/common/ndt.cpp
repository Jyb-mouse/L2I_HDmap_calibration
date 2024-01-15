#include "L2I_calibration/ndt.h"
#include "L2I_calibration/kdtree.h"
#include "L2I_calibration/icp.h"
#include <glog/logging.h>
#include <Eigen/SVD>
#include <execution>
#include <random>
void Ndt::BuildVoxels() {
    assert(target_ != nullptr);
    assert(target_ -> points.size() != 0);
    grids_.clear();

    std::vector<size_t> index(target_ -> size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable {i = idx++;});

    std::for_each(index.begin(), index.end(), [this](const size_t& idx){
        auto pt = ToVec3d(target_ -> points[idx]);
        auto key = (pt * options_.inv_voxel_size_).cast<int>();
        if (grids_.find(key) == grids_.end())
        {
            grids_.insert({key, {idx}});
        }else {
            grids_[key].idx_.emplace_back(idx);
        }
    });

    std::for_each(std::execution::par_unseq, grids_.begin(), grids_.end(), [this](auto& v){
        if (v.second.idx_.size() > options_.min_pts_in_voxel_)
        {
            ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                              [this](const size_t& idx) {return ToVec3d(target_-> points[idx]); });

            Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3)
            {
                lambda[1] = lambda[0] * 1e-3;
            }
            if (lambda[2] < lambda[0] * 1e-3)
            {
                lambda[2] = lambda[0] * 1e-3;
            }
            Eigen::Matrix3d inv_lambda = Eigen::Vector3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
            v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });
    
    for (auto iter = grids_.begin(); iter != grids_.end(); )
    {
        if (iter -> second.idx_.size() > options_.min_pts_in_voxel_)
        {
            iter++;
        }else{
            iter = grids_.erase(iter);
        }   
    }
}
void Ndt::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER)
    {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6)
    {
        nearby_grids_ = {KeyType(0,0,0), KeyType(-1,0,0), KeyType(1,0,0), KeyType(0,1,0),
                        KeyType(0,-1,0), KeyType(0,0,-1), KeyType(0,0,1)};
    }
}
    
bool Ndt::AlignNdt(Eigen::Matrix4d& init_pose) {
    LOG(INFO) << "aligning with ndt";
    assert(grids_.empty() == false);
    Eigen::Matrix4d pose = init_pose;
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_real_distribution<double> distribution(0.0, 0.05);
    // double random_number = distribution(gen);
    // pose(2,3) += random_number;
    if (options_.remove_centroid_)
    {
        pose.block<3,1>(0,3) = target_center_ - source_center_;
        LOG(INFO) << "init trans set to " << pose.block<3,1>(0,3).transpose();
    }
    
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6)
    {
        num_residual_per_point = 7;
    }
    std::vector<int> index(source_ -> points.size());
    for (int i = 0; i < index.size(); i++)
    {
        index[i] = i;
    }

    int total_size = index.size() * num_residual_per_point;
    for (int iter = 0; iter < options_.max_iteration_; iter++)
    {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3,6>> jacobians(total_size);
        std::vector<Eigen::Vector3d> errors(total_size);
        std::vector<Eigen::Matrix3d> infos(total_size);

        // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx){
        for(int idx = 0; idx < index.size(); idx++){ 
            auto q = ToVec3d(source_ -> points[idx]);
            Eigen::Vector3d qs = pose.block<3,3>(0,0) * q + pose.block<3,1>(0,3);
            Eigen::Vector3i key = (qs * options_.inv_voxel_size_).cast<int>();

            for (int i = 0; i < nearby_grids_.size(); i++)
            {
                auto key_off = key + nearby_grids_[i];
                auto it = grids_.find(key_off);
                int real_idx = idx * num_residual_per_point + i;
                if (it != grids_.end())
                {
                    auto& v = it -> second;
                    Eigen::Vector3d e = qs - v.mu_;

                    double res = e.transpose() * v.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_)
                    {
                        effect_pts[real_idx] = false;
                        continue;
                    }
                    
                    Eigen::Matrix<double, 3,6> J;
                    Eigen::Matrix3d q_hat;
                    q_hat << 0,-q[2], q[1], q[2], 0, -q[0], -q[1], q[0], 0;
                    J.block<3,3>(0,0) = - pose.block<3,3>(0,0) * q_hat;
                    J.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                }else{
                    effect_pts[real_idx] = false;
                }
            }
        };

        double total_res = 0;
        int effective_num = 0;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vec6d err = Vec6d::Zero();
        
        for (int idx = 0; idx < effect_pts.size(); idx++)
        {
            if (!effect_pts[idx])
            {
                continue;
            }
            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;
            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }
        
        if (effective_num < options_.min_effective_pts_)
        {
            LOG(WARNING) << "Effective num too small: " << effective_num;
            return false; 
        }

        Vec6d dx = H.inverse() * err;

        Eigen::Vector3d r = dx.head<3>();
        r[2] = 0;
        
        double r2 = r.squaredNorm();
        Eigen::AngleAxisd r_axisd(sqrt(r2), r / sqrt(r2)); 
        Eigen::Matrix3d rotation = r_axisd.toRotationMatrix();
        Eigen::Matrix3d new_R = pose.block<3,3>(0,0) * rotation;
        pose.block<3,3>(0,0) = new_R;
        pose(2,3) += dx(5);
        
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm() 
                  << ", dx: " << dx.transpose();
        
        if (dx.norm() < options_.eps_)
        {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    
    }

    init_pose = pose;
    return true;
}