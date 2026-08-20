#pragma once

#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include <nlohmann/json.hpp>

class ISensorModel {
public:
    virtual ~ISensorModel() = default;
    
    // Inicjalizacja z konfiguracji z config.json
    virtual void load_config(const nlohmann::json& config) = 0;
    
    // Wywoływane co timestep z idealnym stanem z symulatora
    virtual void update(double dt, double current_time, const RocketState& true_state) = 0;
};
