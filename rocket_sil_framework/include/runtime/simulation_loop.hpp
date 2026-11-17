#pragma once
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include <cstdint>
#include <functional>

namespace simrocket {
struct LoopOptions {
    double base_dt;
    uint64_t max_steps;
    double duration_s;
    double real_time_factor;
};
// Does not own models or know their types. The callback executes one base tick.
void runSimulationLoop(RocketState& state, const LoopOptions& options,
                       const std::function<void(double, double)>& tick);
}
