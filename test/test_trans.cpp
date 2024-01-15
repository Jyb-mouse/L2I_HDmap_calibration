#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>




void pose_estimation_3d3d(const std::vector<Eigen::Vector3d> &pts1,
                          const std::vector<Eigen::Vector3d> &pts2,
                          Eigen::Matrix3d &R, Eigen::Vector3d &t) {
  Eigen::Vector3d p1, p2;     // center of mass
  int N = pts1.size();
  for (int i = 0; i < N; i++) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = Eigen::Vector3d(p1 / N);
  p2 = Eigen::Vector3d(p2 / N);
  std::vector<Eigen::Vector3d> q1(N), q2(N); // remove the center
  for (int i = 0; i < N; i++) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; i++) {
    W += Eigen::Vector3d(q1[i][0], q1[i][1], q1[i][2]) * Eigen::Vector3d(q2[i][0], q2[i][1], q2[i][2]).transpose();
  }
  // std::cout << "W=" << W << std::endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // std::cout << "U=" << U << std::endl;
  // std::cout << "V=" << V << std::endl;

  Eigen::Matrix3d R_ = U * (V.transpose());
  if (R_.determinant() < 0) {
    R_ = -R_;
  }
  Eigen::Vector3d t_ = p1 - R_ * p2;

  // convert to cv::Mat
  R = R_;
  t = t_;
}

int main(int argc, char** argv){
    std::vector<Eigen::Vector3d> pts1{Eigen::Vector3d(-0.928000, -2.516000, -0.481000),Eigen::Vector3d(-1.058000, -2.510000, -0.475000), Eigen::Vector3d(-1.068000, -2.508000, -0.580000)};
    std::vector<Eigen::Vector3d> pts2{Eigen::Vector3d(-0.065, -0.055, 0.055), Eigen::Vector3d(0.065, -0.055, 0.055), Eigen::Vector3d(0.065, -0.065, -0.055)};
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    pose_estimation_3d3d(pts1, pts2, R, t);

    std::cout << "R = " << R << std::endl;
    std::cout << "t = " << t.transpose() << std::endl;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;

    Eigen::Vector4d point_lidar;
    point_lidar <<-0.419000, -3.784000, 0.316000, 1;
    Eigen::Vector4d new_point_lidar = T.inverse() * point_lidar;
    std::cout << "point_lidar = " << new_point_lidar.transpose() << std::endl;
    Eigen::Matrix4d T_top2imu = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t_t2i;
    t_t2i << -(0.18367362358270173/2), -(0.17421251390184356/2), 0.027445; 
    T_top2imu.block<3,1>(0,3) = t_t2i;
    std::cout << "T_top2imu = " << T_top2imu << std::endl;
    Eigen::Matrix4d T_s2top = Eigen::Matrix4d::Identity();
    T_s2top = T_top2imu * T.inverse();
    std::cout << "T_s2top = " << T_s2top << std::endl;
    Eigen::Vector4d top;
    top << 5.234, -3.904, 0.09, 1;
    Eigen::Vector4d new_top = T_s2top * top;
    Eigen::Vector4d qj;
    qj << 3.838,-2.758, -0.633, 1;
    Eigen::Vector4d new_qj = T_s2top * qj;
    std::cout << "point_qj = " << new_qj.transpose() << std::endl;
    std::cout << "point_top = " << new_top.transpose() << std::endl;
    std::cout << "res = " << new_top.transpose()-new_qj.transpose() << std::endl;
}