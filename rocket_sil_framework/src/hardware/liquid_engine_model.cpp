#include "rocket_sil_framework/include/hardware/liquid_engine_model.hpp"
#include <algorithm>

LiquidEngineModel::LiquidEngineModel(uint32_t engine_id, double max_thrust_vac_n, double max_thrust_sl_n, double max_mass_flow_kg_s, std::shared_ptr<MessageBus> bus)
    : engine_id_(engine_id), 
      max_thrust_vac_n_(max_thrust_vac_n), 
      max_thrust_sl_n_(max_thrust_sl_n), 
      max_mass_flow_kg_s_(max_mass_flow_kg_s),
      bus_(bus),
      current_throttle_(0.0),
      is_active_(false) {
          
    if (bus_) {
        bus_->subscribe<EngineCommandMsg>([this](const EngineCommandMsg& msg) {
            this->onEngineCommand(msg);
        });
    }
}

LiquidEngineModel::~LiquidEngineModel() {
    // MessageBus currently does not support unsubscribing
}

void LiquidEngineModel::onEngineCommand(const EngineCommandMsg& msg) {
    if (msg.engine_id == engine_id_) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        is_active_ = msg.is_active;
        current_throttle_ = std::clamp(msg.throttle, 0.0, 1.0);
    }
}

EngineOutput LiquidEngineModel::compute(double time_s, const MassProperties& mass_props, double ambient_pressure_pa) {
    EngineOutput out;
    out.thrust_body.setZero();
    out.torque_body.setZero();
    out.mass_flow_rate = 0.0;

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_active_ || current_throttle_ <= 0.0) {
        return out;
    }

    // Prosta interpolacja ciągu zależna od ciśnienia (zakładamy 101325 Pa jako poziom morza, 0 Pa jako próżnia)
    double pressure_ratio = std::clamp(ambient_pressure_pa / 101325.0, 0.0, 1.0);
    double max_thrust_at_alt = max_thrust_vac_n_ + pressure_ratio * (max_thrust_sl_n_ - max_thrust_vac_n_);

    // Idealna odpowiedź przepustnicy (bez uwzględniania turbopompy)
    double current_thrust = max_thrust_at_alt * current_throttle_;

    out.thrust_body = Eigen::Vector3d(0.0, 0.0, current_thrust);
    out.mass_flow_rate = max_mass_flow_kg_s_ * current_throttle_;
    
    return out;
}
