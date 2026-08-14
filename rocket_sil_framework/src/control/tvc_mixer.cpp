#include "rocket_sil_framework/include/control/tvc_mixer.hpp"

TvcMixer::TvcMixer(std::shared_ptr<MessageBus> bus, const std::vector<uint32_t>& active_actuators)
    : bus_(bus), active_actuators_(active_actuators) {
    if (bus_) {
        bus_->subscribe<TvcLogicalCommand>([this](const TvcLogicalCommand& msg) {
            this->allocate(msg.pitch_angle_rad, msg.yaw_angle_rad);
        });
    }
}

void TvcMixer::allocate(double pitch_rad, double yaw_rad) {
    if (!bus_) return;
    
    // Simple allocation: send the exact same angle to all configured TVC actuators.
    // This assumes all engines gimbal together in the same direction.
    for (uint32_t actuator_id : active_actuators_) {
        ActuatorCommandMessage act_msg;
        act_msg.actuator_id = actuator_id;
        act_msg.pitch_angle_rad = pitch_rad;
        act_msg.yaw_angle_rad = yaw_rad;
        bus_->publish(act_msg);
    }
}
