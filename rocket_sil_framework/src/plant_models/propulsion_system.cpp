#include "rocket_sil_framework/include/plant_models/propulsion_system.hpp"
#include <iostream>

PropulsionSystem::PropulsionSystem(std::vector<std::unique_ptr<IEngineModel>> engines,
        std::vector<std::unique_ptr<IActuatorModel>> actuators)
    : engines_(std::move(engines)), actuators_(std::move(actuators)) {}
void PropulsionSystem::advanceActuators(double dt) {
    if (!externally_scheduled_) for (auto& actuator : actuators_) actuator->update(dt);
}
double PropulsionSystem::massFlow(double time, const MassProperties& mass, double pressure) const {
    double total = 0;
    for (const auto& engine : engines_) total += engine->compute(time, mass, pressure).mass_flow_rate;
    return total;
}
void PropulsionSystem::accumulate(double time, const MassProperties& mass, double pressure,
        Eigen::Vector3d& force, Eigen::Vector3d& moment, RocketDiagnostics& diagnostics) const {
    diagnostics.engine_thrust_body.clear();
    diagnostics.thrust_body.setZero();
    for (const auto& engine : engines_) {
        const auto output = engine->compute(time, mass, pressure);
        const IActuatorModel* matched = nullptr;
        for (const auto& actuator : actuators_) if (actuator->getEngineId() == engine->getEngineId()) {
            matched = actuator.get(); break;
        }
        if (!matched) {
            std::cerr << "Warning: Engine " << engine->getEngineId() << " has no matched actuator!" << std::endl;
            continue;
        }
        const auto transform = externally_scheduled_ ? transforms_.at(matched->getActuatorId()) : matched->getTransform();
        const Eigen::Vector3d thrust = transform.transformVectorToParent(output.thrust_body);
        const Eigen::Vector3d lever = transform.getOriginInParent() - mass.center_of_gravity;
        force += thrust;
        moment += lever.cross(thrust);
        diagnostics.thrust_body += thrust;
        diagnostics.engine_thrust_body[engine->getEngineId()] = thrust;
    }
}
