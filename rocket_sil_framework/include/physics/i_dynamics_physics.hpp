#pragma once
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "rocket_sil_framework/include/core/rocket_inputs.hpp"
#include "rocket_sil_framework/include/core/rocket_diagnostics.hpp"

// Explicit boundary between rigid-body integration and physical subsystems.
struct DynamicsEvaluation {
    double mass;
    Eigen::Matrix3d inertia, inertia_inv;
    Eigen::Vector3d force_body, moment_at_cg_body, gravity_world;
    RocketDiagnostics diagnostics;
};
struct PhysicsStep {
    double powered_duration;
    double mass_flow_kg_s;
};
class IDynamicsPhysics {
public:
    virtual ~IDynamicsPhysics() = default;
    virtual PhysicsStep prepareStep(double dt, const RocketState&, const RocketInputs&) = 0;
    virtual void advanceActuators(double dt) = 0;
    virtual void consumePropellant(double mass_flow_kg_s, double dt) = 0;
    // Called at every RK stage. Must not advance mass or actuator state.
    virtual DynamicsEvaluation evaluate(const RocketState&, const RocketInputs&) const = 0;
};
