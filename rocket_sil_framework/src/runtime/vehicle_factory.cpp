#include "rocket_sil_framework/include/runtime/vehicle_factory.hpp"
#include "rocket_sil_framework/include/plant_models/liquid_engine_model.hpp"
#include "rocket_sil_framework/include/plant_models/solid_motor_model.hpp"
#include "rocket_sil_framework/include/plant_models/thrust_curve.hpp"
#include <iostream>
#include <stdexcept>

simrocket::VehicleModels simrocket::buildVehicleModels(
    const nlohmann::json& config, const std::shared_ptr<MessageBus>& message_bus) {
    VehicleModels result;
    const bool component_mode = config.contains("component_runtime");
    if (config["rocket"].contains("engines")) {
        for (const auto& eng_cfg : config["rocket"]["engines"]) {
            uint32_t id = eng_cfg.value("engine_id", 0);
            std::string engine_type = eng_cfg.value("type", "solid");

            if (engine_type == "liquid") {
                double vac_thrust = eng_cfg.value("max_thrust_vac_n", 0.0);
                double sl_thrust = eng_cfg.value("max_thrust_sl_n", 0.0);
                double max_mass_flow = eng_cfg.value("max_mass_flow_kg_s", 0.0);

                auto liquid_eng = std::make_unique<LiquidEngineModel>(id, vac_thrust, sl_thrust, max_mass_flow, message_bus);

                result.engines.push_back(std::move(liquid_eng));
            } else {
                double prop_mass = eng_cfg.value("propellant_mass_kg", 0.0);

                auto curve = std::make_shared<ThrustCurve>();
                std::string profile = eng_cfg.value("thrust_profile", "constant");

                if (profile == "curve") {
                    std::string curve_file = eng_cfg.value("curve_file", "");
                    if (!curve->loadFromFile(curve_file)) {
                        std::cerr << "[Error] Blad wczytywania krzywej ciagu z pliku: " << curve_file << ". Plik nie istnieje lub jest uszkodzony." << std::endl;
                        throw std::runtime_error("Cannot load thrust curve: " + curve_file);
                    }
                } else {
                    double burn_time = eng_cfg.value("burn_time_s", 0.0);
                    double thrust_n = eng_cfg.value("thrust_n", 0.0);
                    curve->addPoint(0.0, thrust_n);
                    curve->addPoint(burn_time, thrust_n);
                    curve->calculateProperties();
                }

                result.engines.push_back(std::make_unique<SolidMotorModel>(id, curve, prop_mass));
            }
        }
    }

    if (config["rocket"].contains("actuators")) {
        for (const auto& act_cfg : config["rocket"]["actuators"]) {
            uint32_t act_id = act_cfg.value("actuator_id", 0);
            uint32_t eng_id = act_cfg.value("engine_id", 0);
            Eigen::Vector3d pos(
                act_cfg["position_m"][0].get<double>(),
                act_cfg["position_m"][1].get<double>(),
                act_cfg["position_m"][2].get<double>()
            );
            auto actuator = std::make_unique<TvcActuatorModel>(act_id, eng_id, pos, component_mode ? nullptr : message_bus);
            if (component_mode) {
                auto initial = config["actuator_initial"]["initial_state"];
                auto target = config["actuator_initial"]["initial_setpoint"];
                actuator->initialize(initial["angle_x_rad"], initial["angle_y_rad"], target["angle_x_rad"], target["angle_y_rad"]);
            }
            result.actuator_views.push_back(actuator.get());
            result.actuators.push_back(std::move(actuator));
            result.actuator_ids.push_back(act_id);
        }
    }

    return result;
}
