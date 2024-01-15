#include "transform/enu_transform.h"

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <math.h>


EnuTransform::EnuTransform(std::string target_crs, double base_lat_deg, double base_lon_deg, int utm_zone)
        : target_crs_(target_crs),
        base_lat_deg_(base_lat_deg),
        base_lon_deg_(base_lon_deg),
        utm_zone_(utm_zone){
    C_  = proj_context_create();

    P_ = proj_create_crs_to_crs(C_, "EPSG:4326", "+proj=utm +zone=51 +datum=WGS84", NULL);

    if (0 == P_)    
    {
        throw std::runtime_error("Failed to create transformation object.\n");
    }
    PJ* norm = proj_normalize_for_visualization(C_, P_);
    if (0 == norm) {
        throw std::runtime_error("Failed to normalize transformation object.\n");
    }
    proj_destroy(P_);
    P_ = norm;
}

EnuTransform::~EnuTransform() {
  proj_destroy(P_);
  proj_context_destroy(C_);
}

double EnuTransform::get_yaw_bias(double lat_deg, double lon_deg) const {
    double lon_diff = (lon_deg - (6 * utm_zone_ - 183)) / 180.0 * M_PI;
    double lat = lat_deg / 180.0 * M_PI;
    double diff = 180 / M_PI * atan(tan(lon_diff) * sin(lat));
    // proj_destroy(P);
    return diff;
}

Eigen::Vector3d EnuTransform::LLA2XYZ(const Eigen::Vector3d& lla) const {
    PJ_COORD value_wgs84 = proj_coord(lla[1], lla[0], lla[2], 0);
    PJ_COORD value_epsg32651 = proj_trans(P_, PJ_FWD, value_wgs84);
    Eigen::Vector3d xyz{value_epsg32651.enu.e, value_epsg32651.enu.n, value_epsg32651.enu.u};

    return xyz;
}

Eigen::Vector3d EnuTransform::XYZ2LLA(const Eigen::Vector3d& xyz) const {
    PJ_COORD value_epsg32651 = proj_coord(xyz[0], xyz[1], xyz[2], 0);
    // 将wgs84坐标系下的地理空间坐标 a 转换到utm32651坐标系下
    PJ_COORD value_wgs84 = proj_trans(P_, PJ_INV, value_epsg32651);
    Eigen::Vector3d llh{value_wgs84.xyz.x, value_wgs84.xyz.y, value_wgs84.xyz.z};

    return llh;
}

Eigen::Matrix3d EnuTransform::rpy2rotation(double roll, double pitch, double yaw) {
    double cos_roll = cos(roll);
    double cos_pitch = cos(pitch);
    double cos_yaw = cos(yaw);
    double sin_roll = sin(roll);
    double sin_pitch = sin(pitch);
    double sin_yaw = sin(yaw);

    double p_0_0 = cos_yaw * cos_roll - sin_yaw * sin_pitch * sin_roll;
    double p_1_0 = sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll;
    double p_2_0 = -cos_pitch * sin_roll;
    double p_0_1 = -sin_yaw * cos_pitch;
    double p_1_1 = cos_yaw * cos_pitch;
    double p_2_1 = sin_pitch;
    double p_0_2 = cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll;
    double p_1_2 = sin_yaw * sin_roll - cos_yaw * sin_pitch * cos_roll;
    double p_2_2 = cos_pitch * cos_roll;
    Eigen::Matrix3d rotation;
    rotation << p_0_0,p_0_1,p_0_2,p_1_0,p_1_1,p_1_2,p_2_0,p_2_1,p_2_2;
    return rotation;
}