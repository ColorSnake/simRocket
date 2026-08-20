#pragma once

#include "i_engine_controller.hpp"
#include "rocket_sil_framework/include/hardware/thrust_curve.hpp"
#include "rocket_sil_framework/include/bus/message_bus.hpp"
#include <memory>
#include <string>

class ProfileEngineController : public IEngineController {
public:
    // Initialize controller for a specific engine using an external file (e.g., CSV)
    ProfileEngineController(uint32_t engine_id, const std::string& csv_file_path, std::shared_ptr<MessageBus> bus);
    
    // Initialize controller using a pre-populated curve
    ProfileEngineController(uint32_t engine_id, std::shared_ptr<ThrustCurve> profile_curve, std::shared_ptr<MessageBus> bus);

    ~ProfileEngineController() override = default;

    void update(double time_s) override;

private:
    uint32_t engine_id_;
    std::shared_ptr<ThrustCurve> throttle_profile_;
    std::shared_ptr<MessageBus> bus_;
    double last_throttle_;
};
