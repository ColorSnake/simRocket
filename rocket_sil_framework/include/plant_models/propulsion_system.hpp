#pragma once
#include "i_engine_model.hpp"
#include "i_actuator_model.hpp"
#include "rocket_sil_framework/include/core/rocket_diagnostics.hpp"
#include <memory>
#include <vector>
#include <map>

// Engine/nozzle composition: consumes mass properties, ambient pressure and
// actuator transforms, without owning or querying environment/mass models.
class PropulsionSystem {
    std::vector<std::unique_ptr<IEngineModel>> engines_;
    std::vector<std::unique_ptr<IActuatorModel>> actuators_;
    bool externally_scheduled_ = false;
    std::map<uint32_t, Transform3D> transforms_;
public:
    PropulsionSystem(std::vector<std::unique_ptr<IEngineModel>> engines,
                     std::vector<std::unique_ptr<IActuatorModel>> actuators);
    void setExternalActuatorScheduling(bool enabled) { externally_scheduled_ = enabled; }
    void setActuatorTransform(uint32_t id, const Transform3D& transform) { transforms_[id] = transform; }
    void advanceActuators(double dt);
    double massFlow(double time, const MassProperties&, double ambient_pressure) const;
    void accumulate(double time, const MassProperties&, double ambient_pressure,
                    Eigen::Vector3d& force, Eigen::Vector3d& moment, RocketDiagnostics&) const;
};
