#pragma once

#include "physics_types.hpp"

class IEngineModel {
public:
    virtual ~IEngineModel() = default;
    
    // We assume the engine calculates thrust purely along its local Z-axis.
    // The actuator will handle orienting it.
    virtual EngineOutput compute(double time_s, const MassProperties& mass_props) = 0;
    
    virtual uint32_t getEngineId() const = 0;
};
