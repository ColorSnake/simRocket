#pragma once

#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/actuator_messages.hpp"
#include <memory>
#include <vector>
#include <cstdint>

// Logical message from TVC Controller indicating desired gimbal angles for the whole rocket
struct TvcLogicalCommand {
    double pitch_angle_rad;
    double yaw_angle_rad;
};

class TvcMixer {
public:
    TvcMixer(std::shared_ptr<MessageBus> bus, const std::vector<uint32_t>& active_actuators);
    
    // Updates the mixer. In a real system, it would subscribe to TvcLogicalCommand.
    // For simplicity, we can also expose a direct method.
    void allocate(double pitch_rad, double yaw_rad);

private:
    std::shared_ptr<MessageBus> bus_;
    std::vector<uint32_t> active_actuators_;
};
