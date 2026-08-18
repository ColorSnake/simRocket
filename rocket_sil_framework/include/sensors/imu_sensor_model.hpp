#pragma once
#include "rocket_sil_framework/include/sensors/i_sensor_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include <random>

class ImuSensorModel : public ISensorModel {
public:
    ImuSensorModel(MessageBus& message_bus);
    ~ImuSensorModel() override = default;
    
    void load_config(const nlohmann::json& config) override;
    void update(double dt, double current_time, const RocketState& true_state) override;

private:
    MessageBus& message_bus_;
    
    // Parametry szumów
    double gyro_noise_std_{0.0};
    double gyro_bias_instability_{0.0};
    double accel_noise_std_{0.0};
    double accel_bias_instability_{0.0};
    
    // Bieżące biasy (Random Walk)
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d accel_bias_;
    
    // Generator liczb losowych
    std::default_random_engine random_engine_;
    
    // Wynikowe publikacje
    Eigen::Vector3d apply_noise(const Eigen::Vector3d& true_val, const Eigen::Vector3d& bias, double noise_std);
};
