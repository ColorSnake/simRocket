#include "rocket_sil_framework/include/control/profile_engine_controller.hpp"
#include "rocket_sil_framework/include/messages/engine_messages.hpp"
#include <iostream>

ProfileEngineController::ProfileEngineController(uint32_t engine_id, const std::string& csv_file_path, std::shared_ptr<MessageBus> bus)
    : engine_id_(engine_id), bus_(bus), last_throttle_(-1.0) {
    
    throttle_profile_ = std::make_shared<ThrustCurve>();
    if (!throttle_profile_->loadFromFile(csv_file_path)) {
        std::cerr << "[ProfileEngineController] Failed to load throttle profile from " << csv_file_path << std::endl;
    }
}

ProfileEngineController::ProfileEngineController(uint32_t engine_id, std::shared_ptr<ThrustCurve> profile_curve, std::shared_ptr<MessageBus> bus)
    : engine_id_(engine_id), throttle_profile_(profile_curve), bus_(bus), last_throttle_(-1.0) {
}

void ProfileEngineController::update(double time_s) {
    if (!bus_ || !throttle_profile_) return;
    
    // Wykorzystujemy istniejący moduł interpolacji (ThrustCurve) do przepustnicy
    double current_throttle = throttle_profile_->getThrustAt(time_s);
    
    // Publikacja tylko gdy przepustnica się zmienia, by nie spamować szyny
    // Można też nadawać co klatkę, ale w symulacji przepustnica i tak odczytuje najświeższą wiadomość.
    // Publish continuous command
    EngineCommandMsg msg;
    msg.engine_id = engine_id_;
    msg.throttle = current_throttle;
    msg.is_active = (current_throttle > 0.0);
    
    bus_->publish(msg);
}
