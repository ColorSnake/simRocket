#pragma once

#include "i_engine_model.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include "rocket_sil_framework/include/messages/engine_messages.hpp"
#include <memory>
#include <mutex>

class LiquidEngineModel : public IEngineModel {
public:
    LiquidEngineModel(uint32_t engine_id, double max_thrust_vac_n, double max_thrust_sl_n, double max_mass_flow_kg_s, std::shared_ptr<MessageBus> bus);
    ~LiquidEngineModel() override;

    EngineOutput compute(double time_s, const MassProperties& mass_props, double ambient_pressure_pa = 101325.0) override;
    
    uint32_t getEngineId() const override { return engine_id_; }

private:
    void onEngineCommand(const EngineCommandMsg& msg);

    uint32_t engine_id_;
    double max_thrust_vac_n_;
    double max_thrust_sl_n_;
    double max_mass_flow_kg_s_;
    
    std::shared_ptr<MessageBus> bus_;

    std::mutex state_mutex_;
    double current_throttle_;
    bool is_active_;
};
