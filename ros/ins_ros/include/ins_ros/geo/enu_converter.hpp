#pragma once

#include <Eigen/Core>
#include <GeographicLib/Geocentric.hpp>

#include <cmath>
#include <stdexcept>

namespace ins_ros
{

class ENUConverter
{
public:
    ENUConverter();

    /**
     * @brief Set the origin of the local ENU frame.
     *
     * @param latitude_deg  Geodetic latitude [deg]
     * @param longitude_deg Geodetic longitude [deg]
     * @param altitude_m    Ellipsoidal altitude [m]
     */
    void set_origin(
        double latitude_deg,
        double longitude_deg,
        double altitude_m);

    /**
     * @brief Check whether an ENU origin has been initialized.
     */
    bool initialized() const;

    /**
     * @brief Convert LLA coordinates to local ENU coordinates.
     *
     * @param latitude_deg  Geodetic latitude [deg]
     * @param longitude_deg Geodetic longitude [deg]
     * @param altitude_m    Ellipsoidal altitude [m]
     *
     * @return Position in ENU [m]
     */
    Eigen::Vector3d to_enu(
        double latitude_deg,
        double longitude_deg,
        double altitude_m) const;

    /**
     * @brief Convert ECEF coordinates to local ENU coordinates.
     *
     * @param p_ecef ECEF position [m]
     *
     * @return Position in ENU [m]
     */
    Eigen::Vector3d to_enu(
        const Eigen::Vector3d& p_ecef) const;

    /**
     * @brief Convert LLA coordinates to ECEF.
     *
     * @param latitude_deg  Geodetic latitude [deg]
     * @param longitude_deg Geodetic longitude [deg]
     * @param altitude_m    Ellipsoidal altitude [m]
     *
     * @return ECEF position [m]
     */
    Eigen::Vector3d to_ecef(
        double latitude_deg,
        double longitude_deg,
        double altitude_m) const;

    /**
     * @brief Get the ECEF -> ENU rotation matrix.
     */
    const Eigen::Matrix3d& rotation_ecef_to_enu() const;

    /**
     * @brief Get the ENU origin expressed in ECEF.
     */
    const Eigen::Vector3d& origin_ecef() const;

    double origin_latitude() const;
    double origin_longitude() const;
    double origin_altitude() const;

private:
    GeographicLib::Geocentric earth_;

    bool initialized_;

    double origin_latitude_deg_;
    double origin_longitude_deg_;
    double origin_altitude_m_;

    Eigen::Vector3d origin_ecef_;

    // Rotates vectors from ECEF into the local ENU frame.
    Eigen::Matrix3d R_ecef_to_enu_;
};

} // namespace ins_ros