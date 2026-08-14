#pragma once

#include "rocket_sil_framework/include/math/transform3d.hpp"
#include <cstdint>

class IActuatorModel {
public:
    virtual ~IActuatorModel() = default;
    
    virtual void update(double dt) = 0;
    
    // Returns the current transform of the attached component relative to the mount point (e.g. nozzle rotation)
    virtual Transform3D getTransform() const = 0;
    
    virtual uint32_t getActuatorId() const = 0;
    virtual uint32_t getEngineId() const = 0;
};
