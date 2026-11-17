#pragma once
#include "rocket_sil_framework/include/plant_models/i_engine_model.hpp"
#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <vector>

namespace simrocket {
struct VehicleModels {
    std::vector<std::unique_ptr<IEngineModel>> engines;
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    // Non-owning views remain valid after moving the owners into dynamics.
    std::vector<TvcActuatorModel*> actuator_views;
    std::vector<uint32_t> actuator_ids;
};
VehicleModels buildVehicleModels(const nlohmann::json& config,
                                const std::shared_ptr<MessageBus>& legacy_bus);
}
