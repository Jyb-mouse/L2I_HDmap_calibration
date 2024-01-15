#pragma once

#include <Eigen/Core>

#include <proj.h>
#include <memory>
#include <string>

class EnuTransform
{ 
public:
    typedef std::shared_ptr<EnuTransform> Ptr;
    EnuTransform(std::string target_crs, double base_lat_deg, double base_lon_deg, int utm_zone);
    ~EnuTransform();

    Eigen::Vector3d LLA2XYZ(const Eigen::Vector3d& lla) const;
    Eigen::Vector3d XYZ2LLA(const Eigen::Vector3d& xyz) const;

    Eigen::Matrix3d rpy2rotation(double roll, double pitch, double yaw);
    double get_yaw_bias(double lat_deg, double lon_deg) const;

private:
    const std::string target_crs_;
    double base_lat_deg_;
    double base_lon_deg_;
    int utm_zone_;

    PJ_CONTEXT* C_;
    PJ* P_;
};
