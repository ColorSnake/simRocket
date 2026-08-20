#pragma once

#include "i_controller.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/estimated_state_message.hpp"
#include "rocket_sil_framework/include/control/tvc_mixer.hpp"
#include <memory>
#include <mutex>

class TvcController : public IController {
public:
    TvcController(std::shared_ptr<MessageBus> bus, double kp, double kd, double max_gimbal_rad);
    
    void update(double dt) override;

    // Diagnostics
    double getPitchError() const { return last_error_pitch_; }
    double getYawError() const { return last_error_yaw_; }
    double getCmdPitch() const { return last_cmd_pitch_; }
    double getCmdYaw() const { return last_cmd_yaw_; }

private:
    std::shared_ptr<MessageBus> bus_;
    
    // PID Gains (for simplicity just PD to add damping)
    double kp_;
    double kd_;
    double max_gimbal_rad_;
    
    // Latest state
    std::mutex state_mutex_;
    EstimatedStateMessage latest_state_;
    
    // Diagnostics state
    double last_error_pitch_ = 0.0;
    double last_error_yaw_ = 0.0;
    double last_cmd_pitch_ = 0.0;
    double last_cmd_yaw_ = 0.0;
};
