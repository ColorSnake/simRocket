#include "rocket_sil_framework/include/runtime/simulation_loop.hpp"
#include <chrono>
#include <thread>
#include <iostream>

namespace simrocket {
void runSimulationLoop(RocketState& state, const LoopOptions& options,
                       const std::function<void(double, double)>& tick) {
    const auto dt = options.base_dt;
    const auto max_steps = options.max_steps;
    const auto simulation_time_s = options.duration_s;
    const auto real_time_factor = options.real_time_factor;
    const auto frame_duration = std::chrono::duration<double, std::micro>(dt * 1e6);
    bool running = true;
    bool has_launched = false;
    uint64_t step_count = 0;

    std::cout << "Entering Hot-Loop..." << std::endl;

    // Main Deterministic Lock-Step Loop
    while (running) {
        auto frame_start = std::chrono::steady_clock::now();

        // Wywołanie schedulera (zastępuje całą starą monolityczną strukturę)
        state.time = step_count * dt;
        tick(step_count * dt, dt);

        if (step_count % 1000 == 0) { // Print at 1Hz for debugging
            std::cout << "[simRocket] Time: " << state.time
                      << "s | Pos Z: " << state.position.z() << "m\n";
        }

        // ---------------------------------------------------------
        // 7. Limitacja czasu do czasu rzeczywistego (Real-Time Throttle)
        // ---------------------------------------------------------
        step_count++;

        if (real_time_factor > 0.0) {
            auto frame_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
            auto target_duration = std::chrono::duration<double, std::micro>(frame_duration.count() * real_time_factor);

            if (elapsed < target_duration) {
                std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(target_duration - elapsed));
            }
        }

        // For this demo, stop after the configured simulation time
        if (step_count >= max_steps) {
            std::cout << "[simRocket] Reached configured simulation time of " << simulation_time_s << "s. Ending simulation." << std::endl;
            running = false;
        }

        // Ground collision detection
        if (state.position.z() > 1.0) {
            has_launched = true;
        }

        if (state.position.z() <= 0.0) {
            // Uniemożliwienie zapadania się pod ziemię przed startem
            state.position.z() = 0.0;
            if (state.acceleration.z() < 0.0) state.acceleration.z() = 0.0;
            if (state.velocity.z() < 0.0) {
                state.velocity.z() = 0.0;
            }

            // Jeśli rakieta już wystartowała (była pow. 1 m) i wróciła na ziemię - koniec symulacji
            if (has_launched) {
                std::cout << "[simRocket] Rocket hit the ground (Z <= 0). Ending simulation." << std::endl;
                running = false;
            }
        }
    }

}
}
