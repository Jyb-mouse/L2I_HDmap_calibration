#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <Eigen/Geometry>
#include "common/config.h"
#include "L2I_calibration/calibration.h"
#include "omp.h"

struct Time_Pos
{
    double time;
    Eigen::Matrix4d pose;
};


std::vector<Time_Pos> GetTimeAndGpsPose(std::string config_path){
    Config cfg(config_path);
    L2I_Calibration::Ptr l2i_calibration(new L2I_Calibration(cfg));
    l2i_calibration -> Init();
    std::vector<Time_Pos> res;
    auto start_inter_pose = std::chrono::high_resolution_clock::now();
    std::map<double, Eigen::Matrix4d> time_pose = l2i_calibration -> Run();
    auto end_inter_pose = std::chrono::high_resolution_clock::now();
    auto duration_inter_pose = std::chrono::duration_cast<std::chrono::microseconds>(end_inter_pose - start_inter_pose);
    std::cout << "time of inter pose and lidar undistort: " << duration_inter_pose.count() / 1000 << " ms." << std::endl;
    for (auto it = time_pose.begin(); it!= time_pose.end() ;it++)
    {
        Time_Pos tmp;
        tmp.time = it -> first;
        tmp.pose = it->second;
        res.emplace_back(tmp);
    }
    return res;
}

Time_Pos GetOneTimeAndGpsPose(std::string config_path, double lidar_time){
    Config cfg(config_path);
    L2I_Calibration::Ptr l2i_calibration(new L2I_Calibration(cfg));
    l2i_calibration -> Init();
    Time_Pos res;
    auto start_inter_pose = std::chrono::high_resolution_clock::now();
    std::map<double, Eigen::Matrix4d> time_pose = l2i_calibration -> Run2();
    auto end_inter_pose = std::chrono::high_resolution_clock::now();
    auto duration_inter_pose = std::chrono::duration_cast<std::chrono::microseconds>(end_inter_pose - start_inter_pose);
    std::cout << "time of inter pose and lidar undistort: " << duration_inter_pose.count() / 1000 << " ms." << std::endl;
    auto it = time_pose.begin();
    res.time = it -> first;
    res.pose = it -> second;
    return res;
}

PYBIND11_MODULE(HDmap_L2I_py, m) {

    pybind11::class_<Time_Pos>(m, "Time_Pos")
    .def(pybind11::init<double, Eigen::Matrix4d>())
    .def_readwrite("time", &Time_Pos::time)
    .def_readwrite("pose", &Time_Pos::pose);
    
    m.doc() = " pybind HDmap_L2I";
    // m.def(pybind11::init<std::map<double, Eigen::Matrix4d>>())
    m.def("GetTimeAndGpsPose", &GetTimeAndGpsPose, "Get Time And Gps Pose");
    m.def("GetOneTimeAndGpsPose", &GetOneTimeAndGpsPose, "Get One Time And Gps Pose");
    // py::class_<HDMap::MaxieyeHDMap::ResultSet>;
    // py::class_<ResultSet>(m,"ResultSet")
    // .def(py::init<std::string,std::vector<std::vector<Eigen::Vector3d>>>())
    // .def_readwrite("name",&ResultSet::name)
    // .def_readwrite("points",&ResultSet::points);
    // m.doc() = "pybind11 example plugin"; // optional module docstring
    // m.def("search_example", &search_example, "HDmap search example for python");
    // m.def("GetSearchResult",&GetSearchResult,"GetSearchResult");
    // m.def("GetSearchResultLLA",&GetSearchResultLLA,"GetSearchResultLLA");
}
