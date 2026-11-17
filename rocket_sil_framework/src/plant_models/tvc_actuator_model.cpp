#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"

TvcActuatorModel::TvcActuatorModel(uint32_t actuator_id, uint32_t engine_id, const Eigen::Vector3d& position_m, std::shared_ptr<MessageBus> bus)
    : actuator_id_(actuator_id), engine_id_(engine_id), position_m_(position_m), bus_(bus) {
    if (bus_) {
        subscriptions_.push_back(bus_->subscribeScoped<ActuatorCommandMessage>([this](const ActuatorCommandMessage& msg) {
            if (msg.actuator_id == this->actuator_id_) {
                std::lock_guard<std::mutex> lock(this->state_mutex_);
                this->target_pitch_rad_ = msg.pitch_angle_rad;
                this->target_yaw_rad_ = msg.yaw_angle_rad;
            }
        }));
    }
}

void TvcActuatorModel::update(double dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // In the future, add low-pass filter or slew rate limit here to model servo delay.
    // For now, instant response:
    rate_x_ = dt > 0 ? (target_yaw_rad_ - current_yaw_rad_) / dt : 0;
    rate_y_ = dt > 0 ? (target_pitch_rad_ - current_pitch_rad_) / dt : 0;
    current_pitch_rad_ = target_pitch_rad_;
    current_yaw_rad_ = target_yaw_rad_;
}

Transform3D TvcActuatorModel::getTransform() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Eigen::AngleAxisd pitch_rot(current_pitch_rad_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rot(current_yaw_rad_, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond nozzle_rotation = yaw_rot * pitch_rot;
    return Transform3D(position_m_, nozzle_rotation);
}

void TvcActuatorModel::initialize(double angle_x, double angle_y, double target_x, double target_y) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_yaw_rad_ = angle_x; current_pitch_rad_ = angle_y;
    target_yaw_rad_ = target_x; target_pitch_rad_ = target_y;
}
Eigen::Vector4d TvcActuatorModel::actualState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return Eigen::Vector4d(current_yaw_rad_, current_pitch_rad_, rate_x_, rate_y_);
}
void TvcActuatorModel::setTarget(double angle_x, double angle_y) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_yaw_rad_ = angle_x; target_pitch_rad_ = angle_y;
}
