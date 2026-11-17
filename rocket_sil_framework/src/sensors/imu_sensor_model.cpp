#include "rocket_sil_framework/include/sensors/imu_sensor_model.hpp"
#include "rocket_sil_framework/include/messages/sensor_messages.hpp"
#include <iostream>

ImuSensorModel::ImuSensorModel(MessageBus& message_bus) 
    : message_bus_(message_bus), 
      gyro_bias_(Eigen::Vector3d::Zero()), 
      accel_bias_(Eigen::Vector3d::Zero()) 
{
    random_engine_.seed(1);
}

void ImuSensorModel::load_config(const nlohmann::json& config) {
    random_engine_.seed(config.value("random_seed", 1u));
    gravity_.z() = config.value("environment", nlohmann::json::object()).value("gravity_z", -9.81);
    if (config.contains("sensors") && config["sensors"].contains("imu")) {
        const auto& imu_cfg = config["sensors"]["imu"];
        gyro_noise_std_ = imu_cfg.value("gyro_noise_std_rad_s", 0.0);
        gyro_bias_instability_ = imu_cfg.value("gyro_bias_instability_rad_s2", 0.0);
        accel_noise_std_ = imu_cfg.value("accel_noise_std_m_s2", 0.0);
        accel_bias_instability_ = imu_cfg.value("accel_bias_instability_m_s3", 0.0);
        
        for (double value : {gyro_noise_std_, gyro_bias_instability_, accel_noise_std_, accel_bias_instability_})
            if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid IMU noise");
        std::cout << "[IMU] Loaded config: gyro_std=" << gyro_noise_std_ 
                  << ", accel_std=" << accel_noise_std_ << "\n";
    }
}

Eigen::Vector3d ImuSensorModel::apply_noise(const Eigen::Vector3d& true_val, const Eigen::Vector3d& bias, double noise_std) {
    std::normal_distribution<double> dist(0.0, 1.0);
    Eigen::Vector3d noise(dist(random_engine_), dist(random_engine_), dist(random_engine_));
    return true_val + bias + noise_std * noise;
}

void ImuSensorModel::update(double dt, double current_time, const RocketState& true_state) {
    // 1. Zaktualizuj biasy krokiem Random Walk (Brownian motion)
    std::normal_distribution<double> gyro_rw_dist(0.0, 1.0);
    std::normal_distribution<double> accel_rw_dist(0.0, 1.0);
    
    gyro_bias_ += gyro_bias_instability_ * sqrt(dt) * Eigen::Vector3d(gyro_rw_dist(random_engine_), gyro_rw_dist(random_engine_), gyro_rw_dist(random_engine_));
    accel_bias_ += accel_bias_instability_ * sqrt(dt) * Eigen::Vector3d(accel_rw_dist(random_engine_), accel_rw_dist(random_engine_), accel_rw_dist(random_engine_));
    
    // 2. Dodaj szumy
    Eigen::Vector3d noisy_angular_vel = apply_noise(true_state.angular_velocity, gyro_bias_, gyro_noise_std_);
    Eigen::Vector3d noisy_linear_accel = apply_noise(true_state.orientation.conjugate() * (true_state.acceleration - gravity_), accel_bias_, accel_noise_std_);
    
    // (Można by też dodawać szum do orientation, jeśli traktujemy to jako AHRS, 
    // ale zwykle IMU podaje rate i accel. Na razie przekażemy czystą orientację lub można by też zastosować do niej błędy)
    
    ImuStateMessage imu_msg;
    imu_msg.timestamp_s = current_time;
    imu_msg.available_time_s = current_time;
    imu_msg.orientation = true_state.orientation;
    imu_msg.angular_velocity = noisy_angular_vel;
    imu_msg.linear_acceleration = noisy_linear_accel;
    
    message_bus_.publish(imu_msg);
}
