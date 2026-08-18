#pragma once
#include "rocket_sil_framework/include/sensors/i_sensor_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/sensor_messages.hpp"
#include <queue>
#include <random>

class GpsSensorModel : public ISensorModel {
public:
    GpsSensorModel(MessageBus& message_bus);
    ~GpsSensorModel() override = default;
    
    void load_config(const nlohmann::json& config) override;
    void update(double dt, double current_time, const RocketState& true_state) override;

private:
    struct QueuedGpsData {
        double timestamp_to_publish;
        GpsStateMessage message;
    };

    MessageBus& message_bus_;
    
    // Parametry
    double update_rate_hz_{10.0};
    double delay_ms_{0.0};
    double position_noise_std_m_{0.0};
    
    // Stan
    double last_update_time_{-100.0};
    std::queue<QueuedGpsData> delay_queue_;
    
    // Generator
    std::default_random_engine random_engine_;
    
    // Przeliczenia Ziemi (bardzo uproszczone na potrzeby symulacji)
    double origin_lat_{0.0};
    double origin_lon_{0.0};
    double origin_alt_{0.0};
};
