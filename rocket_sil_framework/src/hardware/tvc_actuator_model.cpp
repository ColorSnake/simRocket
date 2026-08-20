#include "rocket_sil_framework/include/hardware/tvc_actuator_model.hpp"

TvcActuatorModel::TvcActuatorModel(uint32_t actuator_id, uint32_t engine_id, const Eigen::Vector3d& position_m, std::shared_ptr<MessageBus> bus)
    : actuator_id_(actuator_id), engine_id_(engine_id), position_m_(position_m), bus_(bus) {
    if (bus_) {
        bus_->subscribe<ActuatorCommandMessage>([this](const ActuatorCommandMessage& msg) {
            if (msg.actuator_id == this->actuator_id_) {
                std::lock_guard<std::mutex> lock(this->state_mutex_);
                this->target_pitch_rad_ = msg.pitch_angle_rad;
                this->target_yaw_rad_ = msg.yaw_angle_rad;
            }
        });
    }
}

void TvcActuatorModel::update(double dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // In the future, add low-pass filter or slew rate limit here to model servo delay.
    // For now, instant response:
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
