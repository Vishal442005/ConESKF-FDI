#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace coneskf {

// Skew-symmetric matrix operator [v]_x
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0,  -v.z(),  v.y(),
          v.z(),  0.0,  -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

// SO(3) Exponential Map: so(3) -> SO(3) via Rodrigues formula
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-8) {
        return Eigen::Matrix3d::Identity() + skew(omega);
    }
    Eigen::Matrix3d S = skew(omega);
    return Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * S 
           + ((1.0 - std::cos(theta)) / (theta * theta)) * (S * S);
}

// SO(3) Logarithmic Map: SO(3) -> so(3)
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    double cos_theta = 0.5 * (tr - 1.0);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    double theta = std::acos(cos_theta);
    if (theta < 1e-8) {
        return 0.5 * Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    }
    double factor = theta / (2.0 * std::sin(theta));
    return factor * Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
}

// Coarse gravity alignment: finds rotation R0 that maps measured stationary specific force to [0, 0, 1]^T
inline Eigen::Matrix3d computeGravityAlignment(const Eigen::Vector3d& a_init) {
    Eigen::Vector3d v1 = a_init.normalized();
    Eigen::Vector3d v2(0.0, 0.0, 1.0);
    double c = v1.dot(v2);
    if (c < -0.999999) {
        Eigen::Matrix3d R;
        R << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
        return R;
    }
    Eigen::Vector3d v = v1.cross(v2);
    Eigen::Matrix3d vx = skew(v);
    return Eigen::Matrix3d::Identity() + vx + (vx * vx) / (1.0 + c);
}

} // namespace coneskf
