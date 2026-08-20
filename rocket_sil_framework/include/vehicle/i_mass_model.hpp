#pragma once

#include "rocket_sil_framework/include/core/physics_types.hpp"

class IMassModel {
public:
    virtual ~IMassModel() = default;
    
    // Oblicza aktualną masę i bezwładność, odejmując spalone paliwo w danym dt
    virtual void update(double mass_flow_rate, double dt) = 0;
    
    // Pobiera aktualne właściwości masowe (zawsze zaktualizowane po wywołaniu update)
    virtual MassProperties getProperties() const = 0;
};
