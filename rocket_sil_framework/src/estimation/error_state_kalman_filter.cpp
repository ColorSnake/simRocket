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

ErrorStateKalmanFilter::ErrorStateKalmanFilter(std::shared_ptr<MessageBus> bus, const Vector3d& initial_pos, const Quaterniond& initial_ori, const Vector3d& gravity, double dt_s)
    : bus_(bus), dt_(dt_s), gravity_(gravity), initialized_(false) {
    
    p_ = initial_pos;
    v_.setZero();
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
        bus_->subscribe<ImuStateMessage>([this](const ImuStateMessage& msg) {
            this->onImuMessage(msg);
        });
        bus_->subscribe<GpsStateMessage>([this](const GpsStateMessage& msg) {
            this->onGpsMessage(msg);
        });
    }
}

void ErrorStateKalmanFilter::setProcessNoise(double sig_a, double sig_g, double sig_ba, double sig_bg) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Matrix3d::Identity() * (sig_a * sig_a);
    Q_.block<3,3>(3,3) = Matrix3d::Identity() * (sig_g * sig_g);
    Q_.block<3,3>(6,6) = Matrix3d::Identity() * (sig_ba * sig_ba);
    Q_.block<3,3>(9,9) = Matrix3d::Identity() * (sig_bg * sig_bg);
}

void ErrorStateKalmanFilter::setMeasurementNoise(double sig_gps_pos) {
    R_gps_ = Matrix3d::Identity() * (sig_gps_pos * sig_gps_pos);
}

void ErrorStateKalmanFilter::onImuMessage(const ImuStateMessage& msg) {
    predict(msg.linear_acceleration, msg.angular_velocity);
    
    if (bus_) {
        bus_->publish(getEstimatedState());
    }
}

void ErrorStateKalmanFilter::onGpsMessage(const GpsStateMessage& msg) {
    if (!gps_ref_set_) {
        ref_lat_ = msg.latitude;
        ref_lon_ = msg.longitude;
        gps_ref_set_ = true;
    }
    
    double R_earth = 6371000.0;
    
    double dx = (msg.latitude - ref_lat_) * (M_PI / 180.0) * R_earth;
    double dy = (msg.longitude - ref_lon_) * (M_PI / 180.0) * R_earth * cos(ref_lat_ * M_PI / 180.0);
    double dz = msg.altitude_m;
    
    Vector3d measured_pos(dx, dy, dz);
    updateGPS(measured_pos);
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
    P_ = (I - K * H) * P_;
    
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
    msg.position = p_;
    msg.velocity = v_;
    msg.orientation = q_;
    msg.angular_velocity = last_gyro_ - bg_;
    msg.linear_acceleration = last_accel_ - ba_;
    msg.gyro_bias = bg_;
    msg.accel_bias = ba_;
    return msg;
}
