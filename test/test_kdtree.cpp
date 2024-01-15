#include "L2I_calibration/kdtree.h"
#include "common/point.h"

int main(int argc, char** argv){
    PointCloud::Ptr cloud(new PointCloud);
    Point3d p1(0,0,0,0,0,0);
    Point3d p2(1,0,0,0,0,0);
    Point3d p3(0,1,0,0,0,0);
    Point3d p4(1,1,0,0,0,0);

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    KdTree kdtree;
    kdtree.BuildTree(cloud);
    kdtree.PrintAll();
    return 0;
}