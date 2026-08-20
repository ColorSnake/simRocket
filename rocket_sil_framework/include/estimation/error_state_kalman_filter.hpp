#pragma once

#include <Eigen/Dense>
#include <memory>
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/sensor_messages.hpp"
#include "rocket_sil_framework/include/messages/estimated_state_message.hpp"

class ErrorStateKalmanFilter {
public:
    ErrorStateKalmanFilter(std::shared_ptr<MessageBus> bus, const Eigen::Vector3d& initial_pos, const Eigen::Quaterniond& initial_ori, const Eigen::Vector3d& gravity, double dt_s);


    // Jacobians and Noise setup
    void setProcessNoise(double sigma_accel, double sigma_gyro, double sigma_accel_bias, double sigma_gyro_bias);
    void setMeasurementNoise(double sigma_gps_pos);

    // Callbacks processing
    void onImuMessage(const ImuStateMessage& msg);
    void onGpsMessage(const GpsStateMessage& msg);

    EstimatedStateMessage getEstimatedState() const;

private:
    void predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
    void updateGPS(const Eigen::Vector3d& gps_pos);
    void injectErrorState();

    std::shared_ptr<MessageBus> bus_;
    double dt_;
    Eigen::Vector3d gravity_;

    // Nominal State
    Eigen::Vector3d p_; // position
    Eigen::Vector3d v_; // velocity
    Eigen::Quaterniond q_; // orientation
    Eigen::Vector3d bg_; // gyro bias
    Eigen::Vector3d ba_; // accel bias

    // Error State (always resets to 0 after injection)
    Eigen::Matrix<double, 15, 1> dx_;

    // Covariance
    Eigen::Matrix<double, 15, 15> P_;

    // Process Noise Covariance
    Eigen::Matrix<double, 12, 12> Q_;
    
    // Measurement Noise Covariance
    Eigen::Matrix3d R_gps_;
    
    // Last processed inputs
    Eigen::Vector3d last_accel_;
    Eigen::Vector3d last_gyro_;
    bool initialized_;
    bool gps_ref_set_ = false;
    double ref_lat_ = 0.0;
    double ref_lon_ = 0.0;
};
