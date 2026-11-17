#include "rocket_sil_framework/include/estimation/error_state_kalman_filter.hpp"
#include <iostream>

using namespace Eigen;

static Matrix3d skew(const Vector3d& v) {
    Matrix3d S;
    S <<  0.0, -v.z(),  v.y(),
          v.z(),  0.0, -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

// Map angle-axis vector to quaternion
static Quaterniond expMap(const Vector3d& v) {
    double angle = v.norm();
    if (angle < 1e-8) {
        return Quaterniond(1.0, 0.5*v.x(), 0.5*v.y(), 0.5*v.z()).normalized();
    }
    Vector3d axis = v / angle;
    return Quaterniond(AngleAxisd(angle, axis));
}

ErrorStateKalmanFilter::ErrorStateKalmanFilter(std::shared_ptr<MessageBus> bus, const Vector3d& initial_pos, const Quaterniond& initial_ori, const Vector3d& gravity, double dt_s, const Vector3d& initial_velocity)
    : bus_(bus), dt_(dt_s), gravity_(gravity), initialized_(false) {
    
    p_ = initial_pos;
    v_ = initial_velocity;
    q_ = initial_ori;
    q_.normalize();
    bg_.setZero();
    ba_.setZero();
    
    dx_.setZero();
    P_.setIdentity();
    P_.block<3,3>(0,0) *= 1e-2; // Pos
    P_.block<3,3>(3,3) *= 1e-2; // Vel
    P_.block<3,3>(6,6) *= 1e-4; // Ori
    P_.block<3,3>(9,9) *= 1e-6; // BG
    P_.block<3,3>(12,12) *= 1e-4; // BA

    Q_.setIdentity();
    R_gps_.setIdentity();
    R_gps_ *= 1.0; // 1m pos noise

    if (bus_) {
        subscriptions_.push_back(bus_->subscribeScoped<ImuStateMessage>([this](const ImuStateMessage& msg) {
            this->onImuMessage(msg);
        }));
        subscriptions_.push_back(bus_->subscribeScoped<GpsStateMessage>([this](const GpsStateMessage& msg) {
            this->onGpsMessage(msg);
        }));
    }
}

void ErrorStateKalmanFilter::setProcessNoise(double sig_a, double sig_g, double sig_ba, double sig_bg) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Matrix3d::Identity() * (sig_a * sig_a);
    Q_.block<3,3>(3,3) = Matrix3d::Identity() * (sig_g * sig_g);
    Q_.block<3,3>(6,6) = Matrix3d::Identity() * (sig_bg * sig_bg);
    Q_.block<3,3>(9,9) = Matrix3d::Identity() * (sig_ba * sig_ba);
}

void ErrorStateKalmanFilter::setMeasurementNoise(double sig_gps_pos) {
    R_gps_ = Matrix3d::Identity() * (sig_gps_pos * sig_gps_pos);
}

void ErrorStateKalmanFilter::onImuMessage(const ImuStateMessage& msg) {
    if (!std::isfinite(msg.timestamp_s) || msg.timestamp_s < 0 || msg.timestamp_s <= last_imu_time_) return;
    if (last_imu_time_ >= 0) {
        dt_ = msg.timestamp_s - last_imu_time_;
        predict(msg.linear_acceleration, msg.angular_velocity);
    }
    last_imu_time_ = msg.timestamp_s;
    last_accel_ = msg.linear_acceleration;
    last_gyro_ = msg.angular_velocity;
    
    if (bus_) {
        bus_->publish(getEstimatedState());
    }
}

void ErrorStateKalmanFilter::onGpsMessage(const GpsStateMessage& msg) {
    if (msg.available_time_s - msg.timestamp_s > gps_max_age_ + 1e-9) return;
    // First-order latency compensation using the GPS Doppler velocity.
    // This is not a full out-of-sequence EKF replay.
    if (last_imu_time_ < 0 || msg.timestamp_s > last_imu_time_ + 1e-9) return;
    const double age = last_imu_time_ - msg.timestamp_s;
    Eigen::Vector3d velocity_enu(msg.velocity_ned.y(), msg.velocity_ned.x(), -msg.velocity_ned.z());
    updateGPS(msg.position_local + velocity_enu * age);

}

void ErrorStateKalmanFilter::predict(const Vector3d& accel, const Vector3d& gyro) {
    Vector3d a_true = accel - ba_;
    Vector3d w_true = gyro - bg_;
    
    Matrix3d R_mat = q_.toRotationMatrix();
    
    // Nominal state kinematics
    Vector3d p_new = p_ + v_ * dt_ + 0.5 * (R_mat * a_true + gravity_) * dt_ * dt_;
    Vector3d v_new = v_ + (R_mat * a_true + gravity_) * dt_;
    Quaterniond q_new = q_ * expMap(w_true * dt_);
    
    // Jacobians
    Matrix<double, 15, 15> F = Matrix<double, 15, 15>::Identity();
    F.block<3,3>(0,3) = Matrix3d::Identity() * dt_; // dp/dv
    F.block<3,3>(3,6) = -R_mat * skew(a_true) * dt_; // dv/dth
    F.block<3,3>(3,12) = -R_mat * dt_; // dv/dba
    
    Matrix3d w_skew = skew(w_true);
    // Approximate dth/dth as I - [w]x dt
    F.block<3,3>(6,6) = Matrix3d::Identity() - w_skew * dt_;
    F.block<3,3>(6,9) = -Matrix3d::Identity() * dt_; // dth/dbg
    
    // Noise Jacobian V (15x12)
    Matrix<double, 15, 12> V = Matrix<double, 15, 12>::Zero();
    V.block<3,3>(3,0) = -R_mat * dt_; // vel error from accel noise
    V.block<3,3>(6,3) = -Matrix3d::Identity() * dt_; // ori error from gyro noise
    V.block<3,3>(9,6) = Matrix3d::Identity() * dt_; // gyro bias random walk
    V.block<3,3>(12,9) = Matrix3d::Identity() * dt_; // accel bias random walk
    
    // Covariance update
    P_ = F * P_ * F.transpose() + V * Q_ * V.transpose();
    
    // Force symmetry to prevent numerical divergence
    P_ = 0.5 * (P_ + P_.transpose());
    
    p_ = p_new;
    v_ = v_new;
    q_ = q_new;
    q_.normalize();
    
    last_accel_ = accel;
    last_gyro_ = gyro;
}

void ErrorStateKalmanFilter::updateGPS(const Vector3d& gps_pos) {
    // Measurement model: z = p
    // Error measurement: dz = z - p_nom = p_true + dp + v - p_nom = dp + v
    Vector3d z = gps_pos;
    Vector3d z_hat = p_;
    Vector3d y = z - z_hat;
    
    Matrix<double, 3, 15> H = Matrix<double, 3, 15>::Zero();
    H.block<3,3>(0,0) = Matrix3d::Identity();
    
    Matrix3d S = H * P_ * H.transpose() + R_gps_;
    Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
    
    dx_ = K * y;
    
    Matrix<double, 15, 15> I = Matrix<double, 15, 15>::Identity();
    const auto A = (I - K * H).eval();
    P_ = A * P_ * A.transpose() + K * R_gps_ * K.transpose();
    
    // Force symmetry to prevent numerical divergence
    P_ = 0.5 * (P_ + P_.transpose());
    
    injectErrorState();
}

void ErrorStateKalmanFilter::injectErrorState() {
    p_ += dx_.segment<3>(0);
    v_ += dx_.segment<3>(3);
    
    Vector3d dth = dx_.segment<3>(6);
    q_ = q_ * expMap(dth);
    q_.normalize();
    
    bg_ += dx_.segment<3>(9);
    ba_ += dx_.segment<3>(12);
    
    // Reset error state
    dx_.setZero();
}

EstimatedStateMessage ErrorStateKalmanFilter::getEstimatedState() const {
    EstimatedStateMessage msg;
    msg.timestamp_s = std::max(0.0, last_imu_time_);
    msg.position = p_;
    msg.velocity = v_;
    msg.orientation = q_;
    msg.angular_velocity = last_gyro_ - bg_;
    msg.linear_acceleration = last_accel_ - ba_;
    msg.gyro_bias = bg_;
    msg.accel_bias = ba_;
    return msg;
}
