#pragma once

#include <cstdint>

// Message sent by GNC or user to control engine state
struct EngineCommandMsg {
    uint32_t engine_id;
    double throttle;     // 0.0 to 1.0 (some engines might have min throttle > 0, e.g. 0.4)
    bool is_active;      // True to ignite/keep running, False to shut down

    EngineCommandMsg() : engine_id(0), throttle(0.0), is_active(false) {}
    EngineCommandMsg(uint32_t id, double t, bool active)
        : engine_id(id), throttle(t), is_active(active) {}
};
