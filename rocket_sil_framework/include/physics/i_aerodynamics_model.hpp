#pragma once

#include "rocket_sil_framework/include/core/physics_types.hpp"
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "i_environment_model.hpp"

class IAerodynamicsModel {
public:
    virtual ~IAerodynamicsModel() = default;
    
    // Calculates aerodynamic forces (drag, lift) based on the current state and environment
    virtual AeroForces compute(const RocketState& state, const MassProperties& mass_props, const EnvironmentState& env) = 0;
};
