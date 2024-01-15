#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <unordered_map>
#include <unordered_set>    

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common/bag_reader.h"
#include "common/point.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include </usr/local/include/liblas/liblas.hpp>
#include </usr/local/include/liblas/point.hpp>
#include </usr/local/include/liblas/version.hpp>
#include "utils/filter_data.h"

struct PcdNode
{
    int Grid_ID;//网格编号
    int Grid_ID_x;//行号
    int Grid_ID_y;//列号
    PointCloud::Ptr Grid_Point;
    bool added = false;
};

class PCDGrid
{
public:
    typedef std::shared_ptr<PCDGrid> Ptr;
    PCDGrid(const std::string las_path, const int grid_size);
    explicit PCDGrid(const std::string las_path);
    PCDGrid() = default;
    ~PCDGrid() = default;

    void Init(const std::string las_path,const int grid_size, const int seg_num);

    void LoadLasCloud(const std::string las_path, const std::string proto_path);
    void WriteLasCloud(const std::string output_path);
    void write_las(std::string file_path, const PointCloud &pc_out);

    void New_Point_Cloud_Grid(PointCloud::Ptr cloud);
    void GetLocalMap(const std::string map_path,Eigen::Vector3d translation,
                            pcl::PointCloud<RsPointXYZIRT_f>::Ptr &pc_in);
    
    inline void SetGridSize(int grid_size) {grid_size_ = grid_size; }
    inline int GetGridSize() {return grid_size_;}

    PointCloud::Ptr read_las(const std::string las_path);

    inline size_t ToFlatIndex(const Eigen::Array2i& index){
             int result = 0;
            for (int i = 0; i < 8; ++i) { // Assuming 32-bit integers, adjust if needed
                result |= ((index.x() & (1 << i)) << i) | ((index.y() & (1 << i)) << (i + 1));
            }
            return result;
            // return (index.x() + index.y()) * (index.x() + index.y() + 1) / 2 + index.y();
        }

private:
    std::vector<PcdNode> grids_;
    int grid_size_ = 50;
    PointCloud::Ptr clouds_;
    std::string las_path_;
    int seg_num_ = 1;
};