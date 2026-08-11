#pragma once
#include <Eigen/Dense>

struct ImuStateMessage {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity; // Body frame
};
