#include "rocket_sil_framework/include/sensors/gps_sensor_model.hpp"
#include <iostream>

// Approx meters per degree at equator
constexpr double METERS_PER_DEG_LAT = 111320.0; 

GpsSensorModel::GpsSensorModel(MessageBus& message_bus)
    : message_bus_(message_bus) 
{
    std::random_device rd;
    random_engine_.seed(rd());
}

void GpsSensorModel::load_config(const nlohmann::json& config) {
    if (config.contains("sensors") && config["sensors"].contains("gps")) {
        const auto& gps_cfg = config["sensors"]["gps"];
        update_rate_hz_ = gps_cfg.value("update_rate_hz", 10.0);
        delay_ms_ = gps_cfg.value("delay_ms", 150.0);
        position_noise_std_m_ = gps_cfg.value("position_noise_std_m", 2.0);
        
        std::cout << "[GPS] Loaded config: rate=" << update_rate_hz_ 
                  << "Hz, delay=" << delay_ms_ << "ms\n";
    }
    if (config.contains("location")) {
        origin_lat_ = config["location"].value("latitude", 0.0);
        origin_lon_ = config["location"].value("longitude", 0.0);
        origin_alt_ = config["location"].value("altitude_m", 0.0);
    }
}

void GpsSensorModel::update(double dt, double current_time, const RocketState& true_state) {
    // 1. Publikowanie opóźnionych wiadomości (które już "nadeszły")
    while (!delay_queue_.empty() && delay_queue_.front().timestamp_to_publish <= current_time) {
        message_bus_.publish(delay_queue_.front().message);
        delay_queue_.pop();
    }
    
    // 2. Czy powinniśmy wygenerować nowy pomiar?
    double period_s = 1.0 / update_rate_hz_;
    if (current_time - last_update_time_ >= period_s) {
        last_update_time_ = current_time;
        
        std::normal_distribution<double> pos_noise(0.0, position_noise_std_m_);
        
        // Zaszumiona pozycja lokalna
        double local_x = true_state.position.x() + pos_noise(random_engine_); // North
        double local_y = true_state.position.y() + pos_noise(random_engine_); // East
        double local_z = true_state.position.z() + pos_noise(random_engine_); // Down (altitude is -Z)
        
        // Przeliczenie na koordynaty (bardzo prosta aproksymacja płaskiej ziemi wokół origin)
        double lat_offset = local_x / METERS_PER_DEG_LAT;
        double meters_per_deg_lon = METERS_PER_DEG_LAT * cos(origin_lat_ * M_PI / 180.0);
        double lon_offset = local_y / meters_per_deg_lon;
        
        GpsStateMessage msg;
        msg.latitude = origin_lat_ + lat_offset;
        msg.longitude = origin_lon_ + lon_offset;
        msg.altitude_m = origin_alt_ + local_z; // assuming ENU up is Z... wait! Our simulation uses ENU!
        // In simRocket environment, Z is usually up, so altitude is Z.
        // Let's check config.json -> gravity_z is -9.81, so Z is UP.
        
        // Nie zaszumiamy za mocno prędkości (typowo liczone z dopplera)
        msg.velocity_ned = Eigen::Vector3d(
            true_state.velocity.x(),
            true_state.velocity.y(),
            -true_state.velocity.z()
        ); // Convert ENU velocity to NED convention common for GPS
        
        QueuedGpsData qd;
        qd.timestamp_to_publish = current_time + (delay_ms_ / 1000.0);
        qd.message = msg;
        
        delay_queue_.push(qd);
    }
}
