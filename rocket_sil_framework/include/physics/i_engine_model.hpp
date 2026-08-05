#pragma once

#include "physics_types.hpp"

class IEngineModel {
public:
    virtual ~IEngineModel() = default;
    
    // Oblicza aktualny ciąg, moment oraz ubytek masy
    virtual EngineOutput compute(double time_s, const MassProperties& mass_props) = 0;
};
