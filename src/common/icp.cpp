#include <L2I_calibration/icp.h>
#include <execution>
#include <math.h>


void ICP::BuildTargetKdTree(){
    kdtree_ = std::make_shared<KdTree>();
    kdtree_ -> BuildTree(target_);
    kdtree_ -> SetEnableANN();
}

bool ICP::AlignP2P(Eigen::Matrix4d& init_pose) {
    LOG(INFO) << "align with point to point";
    assert(target_ != nullptr && source_ != nullptr);

    Eigen::Matrix4d pose = init_pose;
    if (!option_.use_initial_translation_)
    {
        pose.block<3,1>(0,3) = target_center_ - source_center_;
    }

    std::vector<int> index(source_ -> points.size());
    for (size_t i = 0; i < index.size(); i++)
    {
        index[i] = i;
    }
    
    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double,3,6>> jacobians(index.size());
    std::vector<Eigen::Vector3d> errors(index.size());

    for (int iter = 0; iter < option_.max_iteration_; ++iter)
    {
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx){
        // for (int idx= 0; idx < source_ -> points.size(); idx++)
        // {
            auto q = ToVec3d(source_ -> points[idx]);
            Eigen::Vector3d qs = pose.block<3,3>(0,0) * q + pose.block<3,1>(0,3);
            Point3d p1(qs(0), qs(1), qs(2), source_ -> points[idx].intensity, source_ -> points[idx].timestamp, source_ -> points[idx].ring);
            std::vector<int> nn;
            kdtree_ -> GetClosestPoint(p1, nn, 1);
            if (!nn.empty())
            {
                Eigen::Vector3d p = ToVec3d(target_-> points[nn[0]]);
                Eigen::Vector3d qs1(qs[0], qs[1], qs[2]);
                double dis2 = (p - qs1).squaredNorm();
                if (dis2 > option_.max_nn_distance_)
                {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;
                Eigen::Vector3d qs0(qs[0], qs[1], qs[2]);
                Eigen::Vector3d e = p - qs0;
                Eigen::Matrix<double,3,6> J;
                Eigen::Matrix3d q_hat;
                q_hat << 0,-q[2], q[1], q[2], 0, -q[0], -q[1], q[0], 0;
                J.block<3,3>(0,0) = pose.block<3,3>(0,0) * q_hat;
                J.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();

                jacobians[idx] = J;
                errors[idx] = e;
            }else {
                effect_pts[idx] = false;
            }
        });
        double total_res = 0;
        int effective_num = 0;

        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts,&total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> 
            std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx])
                {
                    return pre;
                } else {
                    total_res += errors[idx].dot(errors[idx]);
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx], 
                                                                       pre.second - jacobians[idx].transpose() * errors[idx]);
                }
            }
        );

        if (effective_num < option_.min_effective_pts_)
        {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;
        Vec6d dx = H.inverse() * err;

        Eigen::Vector3d r = dx.head<3>();
        double r2 = r.squaredNorm();
        Eigen::AngleAxisd r_axisd(sqrt(r2), r / sqrt(r2)); 
        Eigen::Matrix3d rotation = r_axisd.toRotationMatrix();
        Eigen::Matrix3d new_R = pose.block<3,3>(0,0) * rotation;
        pose.block<3,3>(0,0) = new_R;
        pose.block<3,1>(0,3) += dx.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();
        
        if (dx.norm() < option_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}