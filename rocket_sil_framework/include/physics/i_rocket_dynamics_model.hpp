#pragma once

#include "rocket_sil_framework/include/physics/rocket_state.hpp"
#include "rocket_sil_framework/include/physics/rocket_inputs.hpp"
#include "rocket_sil_framework/include/physics/rocket_diagnostics.hpp"

class IRocketDynamicsModel {
public:
    virtual ~IRocketDynamicsModel() = default;
    
    // Updates the rocket state by integrating over dt
    virtual void update(double dt, const RocketInputs& inputs, RocketState& state) = 0;

    // Pobiera najświeższe dane diagnostyczne po wykonaniu kroku
    virtual RocketDiagnostics getDiagnostics() const = 0;
};
