#include "common/pcd_grid.h"
#include "common/point.h"
#include <boost/algorithm/string.hpp>
#include <string>

PCDGrid::PCDGrid(const std::string las_path) : las_path_(las_path){
    grids_.resize(100000);
}

PCDGrid::PCDGrid(const std::string las_path, const int grid_size)
    : las_path_(las_path), grid_size_(grid_size){
    grids_.resize(100000);
}

void PCDGrid::Init(const std::string las_path,const int grid_size, const int seg_num){
    las_path_ = las_path;
    grid_size_ = grid_size;
    seg_num_ = seg_num;
}


bool fileExists(const std::string& filePath){
    std::ifstream infile{ filePath };
    return infile.good();
}

void PCDGrid::write_las(std::string file_path, const PointCloud &pc_out) {
    std::ofstream ofs(file_path.c_str(), std::ios::out | std::ios::binary);  // 打开las文件
    if (!ofs.is_open()) {
        return;
    }
    liblas::Header header;
    header.SetVersionMajor(1);
    header.SetVersionMinor(2);
    static bool flag = true;

    double g_offset_x = pc_out.points[0][0];
    double g_offset_y = pc_out.points[0][1];
    double g_offset_z = pc_out.points[0][2];
    // }

    header.SetOffset(g_offset_x, g_offset_y, g_offset_z);
    header.SetDataFormatId(liblas::ePointFormat3);
    header.SetPointRecordsCount(pc_out.size());
    header.SetScale(0.001, 0.001, 0.001);

    liblas::Writer writer(ofs, header);
    liblas::Point point(&header);

    double Max_x = -0xFFFFFF, Min_x = 0xFFFFFF;
    double Max_y = -0xFFFFFF, Min_y = 0xFFFFFF;
    double Max_z = -0xFFFFFF, Min_z = 0xFFFFFF;

    //std::cout << "size:" << pc_out.points.size() << std::endl;
    for (const auto &tmp_p : pc_out.points) {
        double x = tmp_p[0];
        double y = tmp_p[1];
        double z = tmp_p[2];

        Min_x = Min_x > x ? x : Min_x;
        Min_y = Min_y > y ? y : Min_y;
        Min_z = Min_z > z ? z : Min_z;
        Max_x = Max_x < x ? x : Max_x;
        Max_y = Max_y < y ? y : Max_y;
        Max_z = Max_z < z ? z : Max_z;

        point.SetCoordinates(x, y, z);
        point.SetIntensity(tmp_p.intensity);
        writer.WritePoint(point);
    }

    header.SetMax(Max_x, Max_y, Max_z);
    header.SetMin(Min_x, Min_y, Min_z);

    writer.SetHeader(header);
    writer.WriteHeader();
    ofs.close();
}

void PCDGrid::New_Point_Cloud_Grid(PointCloud::Ptr cloud){
    //栅格点云

    int cloud_x_size=cloud->points.size();
    for (size_t i = 0; i < cloud_x_size; i++)
	{
        int idx = std::floor((cloud->points[i][0] / float(grid_size_)));
        int idy = std::floor((cloud->points[i][1] / float(grid_size_)));
        
        Eigen::Array2i index(idx,idy);
        size_t id = ToFlatIndex(index);
        // int id = idy * div_x + idx;
        grids_[id].Grid_ID = id;
        grids_[id].Grid_ID_x = idx;
        grids_[id].Grid_ID_y = idy;

        if(!grids_[id].added)
            grids_[id].Grid_Point.reset(new PointCloud);
            grids_[id].added = true;
        grids_[id].Grid_Point->push_back(cloud->points[i]);
	}
    // WriteLasCloud(proto_path);
}

void PCDGrid::WriteLasCloud(const std::string output_path){
    for(int i=0;i<grids_.size();i++)
    {
        if (grids_[i].added)
        {
            std::string temppath;
            temppath = output_path + std::to_string(grids_[i].Grid_ID)+"_"+
                        std::to_string(grids_[i].Grid_ID_x)+"_"+std::to_string(grids_[i].Grid_ID_y);
            write_las(temppath+".las",*(grids_[i].Grid_Point));
        }
    }
}

void PCDGrid::GetLocalMap(const std::string map_path,Eigen::Vector3d translation,
                            pcl::PointCloud<RsPointXYZIRT_f>::Ptr &pc_in){
    int idx = std::floor(translation[0] / grid_size_);
    int idy = std::floor(translation[1] / grid_size_);
    pcl::PointCloud<RsPointXYZIRT_f>::Ptr pcd_cloud(new pcl::PointCloud<RsPointXYZIRT_f>);
    std::vector<int> id_list;
    for(int i=-1;i<1;i++){
        for(int j=-1;j<1;j++){
            int x=idx+i;
            int y=idy+j;
            Eigen::Array2i index(x,y);
            size_t id=ToFlatIndex(index);
            std::string filename=map_path  + std::to_string(id) + 
                            "_" + std::to_string(x)+ "_" + std::to_string(y) + ".las";
            LASreadOpener lasrReadOpener;
            LASreader *lasReader = lasrReadOpener.open(filename.c_str(), false);
            if (lasReader != NULL) {
                size_t count = lasReader->header.number_of_point_records;

                for (size_t j = 0; lasReader->read_point() && j < count; ++j) {
                    RsPointXYZIRT_f points;
                    points.x = lasReader->point.get_x() - translation[0];
                    points.y = lasReader->point.get_y() - translation[1];
                    points.z = lasReader->point.get_z() - translation[2];
                    // std::cout << points.x << " " << points.y << " " << points.z << std::endl;
                    points.intensity = lasReader->point.get_intensity();
                    points.ring = 0;
                    points.timestamp=lasReader->point.get_gps_time();
                    pcd_cloud -> points.push_back(points);
                }
                lasReader->close();
            } else {
                lasReader->close();
            }
        }
    }
    // pcl::PointCloud<RsPointXYZIRT_f> cloud;
    pc_in -> width = pcd_cloud->points.size();                                 //设置点云宽度
    pc_in -> height = 1;                                //设置点云高度
    pc_in -> is_dense = false;                          //非密集型
    pc_in -> points.resize(pc_in ->width * pc_in ->height); 

    for (size_t i = 0; i < pc_in ->points.size(); ++i)
    { 
      pc_in ->points[i].x = pcd_cloud->points[i].x ;
      pc_in ->points[i].y =pcd_cloud->points[i].y ;
      pc_in ->points[i].z = pcd_cloud->points[i].z ;
      pc_in ->points[i].intensity = 0;
      pc_in ->points[i].ring = 0;
      pc_in ->points[i].timestamp = 0;
    }

}

void PCDGrid::LoadLasCloud(const std::string las_path, const std::string proto_path){
    std::vector<std::string> filenamelist;
    std::vector<std::string> file_name;
    getFilesList(filenamelist, file_name, las_path, ".las", true);
    PointCloud::Ptr lidar_points(new PointCloud);
    int seg_count = 0;
    
    for(int i=0;i<filenamelist.size();i++){
        // spdlog::info("filename:{}",filenamelist[i]);
        *(lidar_points)+=*(read_las(filenamelist[i]));
        if(seg_count>= seg_num_)
        {
            // spdlog::info("pointcloud size:{}",cloud->points.size());
            New_Point_Cloud_Grid(lidar_points);
            lidar_points.reset(new PointCloud);
            seg_count=0;
        }
        seg_count+=1;
    }
    WriteLasCloud(proto_path);
}

PointCloud::Ptr PCDGrid::read_las(const std::string las_path) 
{
    PointCloud::Ptr lidar_points(new PointCloud);
    LASreadOpener lasrReadOpener;
    LASreader *lasReader = lasrReadOpener.open(las_path.c_str(), false);
    if (lasReader != NULL) {
    size_t count = lasReader->header.number_of_point_records;
    lidar_points->reserve(count);
    for (size_t j = 0; lasReader->read_point() && j < count; ++j) {
        Point3d point;
        point[0] = lasReader->point.get_x();
        point[1] = lasReader->point.get_y();
        point[2] = lasReader->point.get_z();
        point.intensity=lasReader->point.get_intensity();
        point.ring = 0;
        point.timestamp=lasReader->point.get_gps_time();
        if (!std::isfinite (point[0]) || 
                            !std::isfinite (point[1]) || 
                            !std::isfinite (point[2]))
                            continue;
        if (std::isnan (point[0]) || 
                            std::isnan (point[1]) || 
                            std::isnan (point[2]))
                            continue;

        lidar_points->points.push_back(point);
    }
    lasReader->close();
  } else {
    spdlog::info("read las file_path fail! {}", las_path);
    lasReader->close();
  }
  return lidar_points;
}