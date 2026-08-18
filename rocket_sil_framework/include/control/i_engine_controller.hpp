#pragma once

#include <cstdint>

class IEngineController {
public:
    virtual ~IEngineController() = default;

    // Called every simulation tick. Time_s is the absolute simulation time.
    virtual void update(double time_s) = 0;
};
