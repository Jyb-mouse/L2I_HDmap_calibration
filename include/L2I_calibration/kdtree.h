#pragma once

#include <glog/logging.h>
#include <map>
#include <queue>
#include <numeric>
#include "common/point.h"

template <typename C, typename D, typename Getter>
inline void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
    // clang-format on
}

struct KdTreeNode
{
    int id_ = -1;
    int point_idx_ = 0;
    int axis_index_ = 0;
    float split_thresh_ = 0.0;
    KdTreeNode* left_ = nullptr;
    KdTreeNode* right_ = nullptr;

    bool IsLeaf() const {return left_ == nullptr && right_ == nullptr; }
};

struct NodeAndDistance
{
    NodeAndDistance(KdTreeNode* node, float dis2) : node_(node), distance2_(dis2){}
    KdTreeNode* node_ = nullptr;
    float distance2_ = 0;

    bool operator < (const NodeAndDistance& other) const {return distance2_ < other.distance2_ ;}
};

class KdTree
{
public:
    explicit KdTree() = default;
    ~KdTree() {Clear();}

    bool BuildTree(const PointCloud::Ptr & cloud);

    bool GetClosestPoint(const Point3d& pt, std::vector<int>& closest_idx, int k = 5);

    bool GetClosestPointMT(const PointCloud::Ptr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);
    
    void SetEnableANN(bool use_ann = true, float alpha = 0.1){
        approximate_ = use_ann;
        alpha_ = alpha;
    }
    
    size_t size() const {return size_;}

    void PrintAll();
    void Clear();

private:
    void Insert(const std::vector<int>& points, KdTreeNode* node);

    bool FindSplitAxisAndThresh(const std::vector<int>& point_idx, int& axis, float& th, 
                            std::vector<int>& left, std::vector<int>& right);
    
    void Reset();

    static inline float Dis2(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return (p1 - p2).squaredNorm();
    }

    void Knn(const Eigen::Vector3d& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    void ComputeDisForLeaf(const Eigen::Vector3d& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    bool NeedExpand(const Eigen::Vector3d& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

    int k_ = 5;
    std::shared_ptr<KdTreeNode> root_  = nullptr;
    std::vector<Eigen::Vector3d> cloud_;
    std::unordered_map<int, KdTreeNode*> nodes_;
    
    size_t size_ = 0;
    int tree_node_id_ = 0;

    bool approximate_ = true;
    float alpha_ = 0.1;
};

