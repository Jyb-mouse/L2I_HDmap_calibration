
#include "L2I_calibration/kdtree.h"

#include <glog/logging.h>
#include <vector>
#include <execution>
#include <set>
#include <numeric>

bool KdTree::BuildTree(const PointCloud::Ptr &cloud){
    // if (cloud -> empty())
    // {
    //     return false;
    // }

    cloud_.clear();
    cloud_.resize(cloud -> size());
    for (size_t i = 0; i < cloud -> points.size(); i++)
    {
        cloud_[i] = ToVec3d(cloud -> points[i]);
    }

    Clear();
    Reset();

    std::vector<int> idx(cloud -> size());
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        idx[i] = i;
    }
    Insert(idx, root_.get());
    return true;
}

void KdTree::Insert(const std::vector<int> &points, KdTreeNode *node){
    nodes_.insert({node -> id_, node});
    if (points.empty())
    {
        return;
    }
    if (points.size() == 1)
    {
        size_++;
        node->point_idx_ = points[0];
        return;
    }
    std::vector<int> left, right;
    if (!FindSplitAxisAndThresh(points, node -> axis_index_ , node -> split_thresh_, left, right))  
    {
        size_++;
        node -> point_idx_ = points[0];
        return;
    }
    const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const std::vector<int> &index){
        if (!index.empty())
        {
            new_node = new KdTreeNode;
            new_node -> id_ = tree_node_id_++;
            Insert(index, new_node);
        }
    };

    create_if_not_empty(node -> left_, left);
    create_if_not_empty(node -> right_, right);
}

bool KdTree::GetClosestPoint(const Point3d &pt, std::vector<int> &closest_idx, int k){
    if (k > size_)
    {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
        return false;
    }
    k_ = k;
    std::priority_queue<NodeAndDistance> knn_result;

    Knn(ToVec3d(pt), root_.get(), knn_result);
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; i--)
    {
        closest_idx[i] = knn_result.top().node_ -> point_idx_;
        knn_result.pop();
    }
    return true;
}

bool KdTree::GetClosestPointMT(const PointCloud::Ptr &cloud, std::vector<std::pair<size_t, size_t>> & matches, int k){
    matches.resize(cloud->size() * k);

    std::vector<int> index(cloud -> size());
    for (size_t i = 0; i < cloud -> points.size(); i++)
    {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx){
        std::vector<int> closest_idx;
        GetClosestPoint(cloud -> points[idx], closest_idx, k);
        for (int i = 0; i < k; i++)
        {
            matches[idx * k + i].second = idx;
            if (i < closest_idx.size())
            {
                matches[idx * k + i].first = closest_idx[i];
            } else {
                matches[idx * k + i].first = std::numeric_limits<size_t>::max();
            }
        }        
    });

    return true;
}

void KdTree::Knn(const Eigen::Vector3d &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
    if (node -> IsLeaf())
    {
        ComputeDisForLeaf(pt, node, knn_result);
        return;
    }
    KdTreeNode *this_side, *that_side;
    if (pt[node -> axis_index_] < node -> split_thresh_)
    {
        this_side = node -> left_;
        that_side = node -> right_;
    } else {
        this_side = node -> right_;
        that_side = node -> left_;
    }
    Knn(pt, this_side, knn_result);
    if (NeedExpand(pt, node, knn_result))
    {
        Knn(pt, that_side, knn_result);
    }
}

bool KdTree::NeedExpand(const Eigen::Vector3d & pt, KdTreeNode * node, std::priority_queue<NodeAndDistance> &knn_result) const {
    if (knn_result.size() < k_)
    {
        return true;
    }

    if (approximate_)
    {
        float d = pt[node -> axis_index_] - node -> split_thresh_;
        if ((d * d) < knn_result.top().distance2_ * alpha_)
        {
            return true;
        }else{
            return false;
        }
    } else {
        float d = pt[node -> axis_index_] - node -> split_thresh_;
        if ((d * d) < knn_result.top().distance2_)
        {
            return true;
        }else{
            return false;
        }
    }
}

void KdTree::ComputeDisForLeaf(const Eigen::Vector3d& pt, KdTreeNode * node, 
                            std::priority_queue<NodeAndDistance> &knn_result) const {
    float dis2 = Dis2(pt, cloud_[node -> point_idx_]);
    if (knn_result.size() < k_)
    {
        knn_result.emplace(node, dis2);
    } else {
        if (dis2 < knn_result.top().distance2_)
        {
            knn_result.emplace(node, dis2);
            knn_result.pop();
        }
    }
}

void KdTree::Reset() {
    tree_node_id_ = 0;
    root_.reset(new KdTreeNode());
    root_ -> id_ = tree_node_id_++;
    size_ = 0;
}

void KdTree::Clear() {
    for (const auto &np : nodes_)
    {
        if (np.second != root_.get())
        {
            delete np.second;
        }
    }
    nodes_.clear();
    root_ = nullptr;
    size_ = 0;
    tree_node_id_ = 0;
}

void KdTree::PrintAll(){
    for (const auto & np : nodes_)
    {
        auto node = np.second;
        if (node -> left_ == nullptr && node -> right_ == nullptr) 
        {
            LOG(INFO) << "leaf node : " << node -> id_ << ", idx: " << node -> point_idx_;
        } else {
            LOG(INFO) << "node : " << node -> id_ << ", axis: " << node -> axis_index_ << ", th: " << node -> split_thresh_;
        }
    }
}


bool KdTree::FindSplitAxisAndThresh(const std::vector<int> &point_idx, int& axis, float& th,
                                     std::vector<int> &left, std::vector<int> &right){
    Eigen::Vector3d var;
    Eigen::Vector3d mean;
    ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx){return cloud_[idx]; });
    int max_i, max_j;
    var.maxCoeff(&max_i, &max_j);
    axis = max_i;
    th = mean[axis];

    for (const auto & idx : point_idx)
    {
        if (cloud_[idx][axis] < th)
        {
            left.emplace_back(idx);
        } else {
            right.emplace_back(idx);
        }
    }

    if (point_idx.size() > 1 && (left.empty() || right.empty()))
    {
        return false;
    }
    return true;
}


