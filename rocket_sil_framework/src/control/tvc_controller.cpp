#include "rocket_sil_framework/include/control/tvc_controller.hpp"
#include "rocket_sil_framework/include/control/tvc_mixer.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

TvcController::TvcController(std::shared_ptr<MessageBus> bus, double kp, double kd, double max_gimbal_rad)
    : bus_(bus), kp_(kp), kd_(kd), max_gimbal_rad_(max_gimbal_rad) {
    
    latest_state_.orientation.setIdentity();
    latest_state_.angular_velocity.setZero();
    
    if (bus_) {
        bus_->subscribe<ImuStateMessage>([this](const ImuStateMessage& msg) {
            std::lock_guard<std::mutex> lock(this->state_mutex_);
            this->latest_state_ = msg;
        });
    }
}

void TvcController::update(double dt) {
    if (!bus_) return;
    if (dt <= 0.0) return;

    ImuStateMessage state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = latest_state_;
    }

    // Convert quaternion to Euler angles to get pitch and yaw error.
    // In our system, Z is longitudinal (forward).
    // X is yaw axis? No, typical aerospace: rotation about X is Roll, Y is Pitch, Z is Yaw (if X forward).
    // Our convention: Z forward.
    // Rotation about X is pitch, rotation about Y is yaw (or vice versa).
    // Let's use simple vector projection to find pitch/yaw error relative to Z-up (inertial world).
    
    // Rocket's Z axis in the world frame
    Eigen::Vector3d rocket_z_in_world = state.orientation * Eigen::Vector3d::UnitZ();
    
    // The target is to point perfectly UP (World +Z)
    Eigen::Vector3d target_z_in_world = Eigen::Vector3d::UnitZ();
    
    // Error rotation to get from rocket_z back to target_z
    Eigen::Vector3d error_axis = rocket_z_in_world.cross(target_z_in_world);
    double error_norm = error_axis.norm();
    error_norm = std::clamp(error_norm, -1.0, 1.0);
    double error_angle = asin(error_norm);
    
    if (error_angle < 1e-6) {
        error_axis = Eigen::Vector3d::UnitX(); // Arbitrary, doesn't matter if angle is 0
    } else {
        error_axis.normalize();
    }
    
    // This gives us the error in the WORLD frame. We need it in the BODY frame to actuate nozzles.
    Eigen::Vector3d error_axis_body = state.orientation.inverse() * error_axis;
    
    // Now error_axis_body * error_angle gives the orientation error vector in the body frame.
    // X component of this error represents rotation around X (let's say Pitch)
    // Y component represents rotation around Y (let's say Yaw)
    double error_x = error_axis_body.x() * error_angle;
    double error_y = error_axis_body.y() * error_angle;
    
    // Derivative term (D) - trying to zero out angular velocity
    // Angular velocity is already in body frame
    double d_x = -state.angular_velocity.x();
    double d_y = -state.angular_velocity.y();
    
    // Calculate commanded gimbal angles
    // The signs are inverted to provide negative feedback (opposing the error).
    double cmd_x = -(kp_ * error_x + kd_ * d_x);
    double cmd_y = -(kp_ * error_y + kd_ * d_y);
    
    // Clamp to max gimbal angle
    cmd_x = std::clamp(cmd_x, -max_gimbal_rad_, max_gimbal_rad_);
    cmd_y = std::clamp(cmd_y, -max_gimbal_rad_, max_gimbal_rad_);
    
    // Save diagnostics
    last_error_pitch_ = error_y; // Rotation around Y
    last_error_yaw_ = error_x;   // Rotation around X
    last_cmd_pitch_ = cmd_y;
    last_cmd_yaw_ = cmd_x;
    
    // Publish logical command
    TvcLogicalCommand cmd;
    cmd.yaw_angle_rad = cmd_x;   // Let's call rotation about X "yaw" for the nozzle
    cmd.pitch_angle_rad = cmd_y; // and rotation about Y "pitch"
    
    bus_->publish(cmd);
}
