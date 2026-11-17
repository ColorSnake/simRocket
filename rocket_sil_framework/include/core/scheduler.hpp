#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// Fixed-grid scheduler. Callbacks start an interval [time, time + dt).
// The base frequency must be a common multiple of every task frequency.
class Scheduler {
    struct Task {
        std::string name;
        double frequency;
        uint32_t priority;
        std::function<void(double, double)> callback;
    };
    std::vector<Task> tasks_;
    uint64_t tick_ = 0;
public:
    void addTask(const std::string& name, double frequency, uint32_t priority,
                 std::function<void(double, double)> callback) {
        if (!std::isfinite(frequency) || frequency < 0 || !callback)
            throw std::invalid_argument("Invalid scheduler task: " + name);
        for (const auto& task : tasks_)
            if (task.name == name) throw std::invalid_argument("Duplicate task: " + name);
        tasks_.push_back({name, frequency, priority, std::move(callback)});
        std::stable_sort(tasks_.begin(), tasks_.end(), [](const Task& a, const Task& b) {
            return a.priority < b.priority;
        });
    }
    void validate(double base_dt) const {
        if (!std::isfinite(base_dt) || base_dt <= 0) throw std::invalid_argument("Invalid base step");
        for (const auto& task : tasks_) {
            double ticks = task.frequency == 0 ? 1 : 1 / (task.frequency * base_dt);
            if (!std::isfinite(ticks) || ticks < 1 || ticks > 1e9 || std::abs(ticks - std::round(ticks)) > 1e-7)
                throw std::invalid_argument("Task '" + task.name + "': base_rate_hz must be a multiple of task frequency");
        }
    }
    void update(double time, double base_dt) {
        validate(base_dt);
        if (!std::isfinite(time) || std::abs(time - tick_ * base_dt) > 1e-8)
            throw std::invalid_argument("Scheduler clock must advance by exactly one base step");
        for (const auto& task : tasks_) {
            auto ticks = task.frequency == 0 ? 1ULL : static_cast<uint64_t>(std::llround(1 / (task.frequency * base_dt)));
            if (tick_ % ticks == 0) task.callback(time, ticks * base_dt);
        }
        ++tick_;
    }
};
