#include "ins_ros/geo/enu_converter.hpp"

namespace ins_ros
{

ENUConverter::ENUConverter()
    : earth_(GeographicLib::Geocentric::WGS84())
    , initialized_(false)
    , origin_latitude_deg_(0.0)
    , origin_longitude_deg_(0.0)
    , origin_altitude_m_(0.0)
    , origin_ecef_(Eigen::Vector3d::Zero())
    , R_ecef_to_enu_(Eigen::Matrix3d::Identity())
{
}

void ENUConverter::set_origin(
    double latitude_deg,
    double longitude_deg,
    double altitude_m)
{
    origin_latitude_deg_ = latitude_deg;
    origin_longitude_deg_ = longitude_deg;
    origin_altitude_m_ = altitude_m;

    // LLA -> ECEF
    double X, Y, Z;

    earth_.Forward(
        latitude_deg,
        longitude_deg,
        altitude_m,
        X,
        Y,
        Z);

    origin_ecef_ << X, Y, Z;

    // Convert origin coordinates to radians.
    const double lat =
        latitude_deg * M_PI / 180.0;

    const double lon =
        longitude_deg * M_PI / 180.0;

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    /*
     * ECEF -> ENU rotation:
     *
     *       [-sin(lon),  cos(lon),       0]
     * R =   [-sin(lat)cos(lon),
     *        -sin(lat)sin(lon), cos(lat)]
     *       [ cos(lat)cos(lon),
     *         cos(lat)sin(lon), sin(lat)]
     */
    R_ecef_to_enu_ <<
        -sin_lon,
         cos_lon,
         0.0,

        -sin_lat * cos_lon,
        -sin_lat * sin_lon,
         cos_lat,

         cos_lat * cos_lon,
         cos_lat * sin_lon,
         sin_lat;

    initialized_ = true;
}

bool ENUConverter::initialized() const
{
    return initialized_;
}

Eigen::Vector3d ENUConverter::to_ecef(
    double latitude_deg,
    double longitude_deg,
    double altitude_m) const
{
    double X, Y, Z;

    earth_.Forward(
        latitude_deg,
        longitude_deg,
        altitude_m,
        X,
        Y,
        Z);

    return Eigen::Vector3d(X, Y, Z);
}

Eigen::Vector3d ENUConverter::to_enu(
    const Eigen::Vector3d& p_ecef) const
{
    if (!initialized_)
    {
        throw std::runtime_error(
            "ENUConverter: origin has not been initialized");
    }

    return R_ecef_to_enu_ *
           (p_ecef - origin_ecef_);
}

Eigen::Vector3d ENUConverter::to_enu(
    double latitude_deg,
    double longitude_deg,
    double altitude_m) const
{
    const Eigen::Vector3d p_ecef =
        to_ecef(
            latitude_deg,
            longitude_deg,
            altitude_m);

    return to_enu(p_ecef);
}

const Eigen::Matrix3d&
ENUConverter::rotation_ecef_to_enu() const
{
    return R_ecef_to_enu_;
}

const Eigen::Vector3d&
ENUConverter::origin_ecef() const
{
    return origin_ecef_;
}

double ENUConverter::origin_latitude() const
{
    return origin_latitude_deg_;
}

double ENUConverter::origin_longitude() const
{
    return origin_longitude_deg_;
}

double ENUConverter::origin_altitude() const
{
    return origin_altitude_m_;
}

} // namespace ins_ros