#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

class Point3d : public Eigen::Vector3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Point3d() = default;
  Point3d(double _x, double _y, double _z, u_int16_t _intensity, double _timestamp, double _ring)
      : Eigen::Vector3d(_x, _y, _z), intensity(_intensity), timestamp(_timestamp), ring(_ring) {}
  Point3d(Eigen::Vector3d _vec, u_int16_t _intensity, double _timestamp, double _ring)
      : Eigen::Vector3d(_vec), intensity(_intensity), timestamp(_timestamp), ring(_ring) {}
  const Point3d operator-(const Eigen::Vector3d &r) const {
    return Point3d(Eigen::Vector3d::operator-(r), intensity, timestamp, ring);
  }
  const Point3d operator+(const Eigen::Vector3d &r) const {
    return Point3d(Eigen::Vector3d::operator+(r), intensity, timestamp, ring);
  }
//   void transform(const transform::Rigid3d &pose) { Eigen::Vector3d::operator=(pose *(*this)); }

  u_int16_t intensity;
  double timestamp;
  double ring;
};


class PointCloud {
 public:
  typedef std::shared_ptr<PointCloud> Ptr;
  typedef std::shared_ptr<const PointCloud> ConstPtr;

  PointCloud() = default;
  PointCloud(int id) : id_(id) {}

  // template <typename PointT>
  // PointCloud(const pcl::PointCloud<PointT> &pcl_pc, int id = 0) : id_(id)
  // {
  //     convertFromPCL(pcl_pc);
  // }
  // template <typename PointT>
  // void convertFromPCL(const pcl::PointCloud<PointT> &pcl_pc)
  // {
  //     points.reserve(pcl_pc.size());
  //     for (auto &point : pcl_pc.points)
  //     {
  //         points.emplace_back(point.x, point.y, point.z, std::roundl(point.intensity));
  //     }
  // }

  // template <typename PointT>
  // void transformFrom(const pcl::PointCloud<PointT> &pcl_pc, const Eigen::Matrix4d &mat)
  // {
  //     convertFromPCL<PointT>(pcl_pc);
  //     transform(mat);
  // }

//   void transformFrom(const PointCloud &input_pc, const transform::Rigid3d &pose) {
//     id_ = input_pc.id();
//     points = input_pc.points;
//     transform(pose);
//   }

//   PointCloud transformTo(const transform::Rigid3d &pose) const {
//     PointCloud res(id_);
//     res.transformFrom(*this, pose);
//     return res;
//   }

//   void transform(const transform::Rigid3d &pose) {
//     for (auto &point : points) {
//       point.transform(pose);
//     }
//   }

  inline Point3d &operator[](std::size_t i) { return points[i]; }

  inline const Point3d &operator[](std::size_t i) const { return points[i]; }

  inline PointCloud operator+(const PointCloud &pc) const {
    PointCloud pc_res;
    pc_res.points.insert(pc_res.points.end(), pc.points.begin(), pc.points.end());
    return pc_res;
  }

  inline PointCloud& operator=(const PointCloud& other){
    if (this == &other)
    {
      return *this;
    }
    this -> id_ = other.id_;
    this -> points = other.points;
    return *this;
  }

  inline void operator+=(const PointCloud &pc) {
    points.insert(points.end(), pc.points.begin(), pc.points.end());
  }

  inline const int size() const { return points.size(); }

  void clear() { points.clear(); }

  void reserve(const int num) { points.reserve(num); }

  inline void push_back(const Point3d &point) { points.push_back(point); }

  template <typename... _Args>
  inline void emplace_back(_Args &&...__args) {
    points.emplace_back(std::forward<_Args>(__args)...);
  }

  int id() const { return id_; }

  std::vector<Point3d, Eigen::aligned_allocator<Point3d>> points;
  int id_ = 0;
};

inline Eigen::Vector3d ToVec3d(const Point3d& pt) {return Eigen::Vector3d(pt[0], pt[1], pt[2]);}

template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}