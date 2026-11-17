#pragma once

#include "i_actuator_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/actuator_messages.hpp"
#include <memory>
#include <mutex>
#include <Eigen/Dense>

class TvcActuatorModel : public IActuatorModel {
public:
    TvcActuatorModel(uint32_t actuator_id, uint32_t engine_id, const Eigen::Vector3d& position_m, std::shared_ptr<MessageBus> bus);
    
    void update(double dt) override;
    void initialize(double angle_x, double angle_y, double target_x, double target_y);
    Eigen::Vector4d actualState() const;
    void setTarget(double angle_x, double angle_y);
    
    Transform3D getTransform() const override;
    
    uint32_t getActuatorId() const override { return actuator_id_; }
    uint32_t getEngineId() const override { return engine_id_; }

private:
    uint32_t actuator_id_;
    uint32_t engine_id_;
    Eigen::Vector3d position_m_;
    std::shared_ptr<MessageBus> bus_;
    
    mutable std::mutex state_mutex_;
    double target_pitch_rad_ = 0.0;
    double target_yaw_rad_ = 0.0;
    
    double rate_x_ = 0, rate_y_ = 0;
    double current_pitch_rad_ = 0.0;
    double current_yaw_rad_ = 0.0;
    
    // Simple physical model (e.g., max slew rate) could be added here
    std::vector<MessageBus::Subscription> subscriptions_;
};
