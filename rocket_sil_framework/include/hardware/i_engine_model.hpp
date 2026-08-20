#pragma once

#include "rocket_sil_framework/include/core/physics_types.hpp"

class IEngineModel {
public:
    virtual ~IEngineModel() = default;
    
    // We assume the engine calculates thrust purely along its local Z-axis.
    // Computes and returns the thrust and torque applied to the body.
    // Also computes mass flow rate.
    // ambient_pressure_pa: Ciśnienie otoczenia (ważne dla LRE, domyślnie poziom morza)
    virtual EngineOutput compute(double time_s, const MassProperties& mass_props, double ambient_pressure_pa = 101325.0) = 0;
    
    virtual uint32_t getEngineId() const = 0;
};
