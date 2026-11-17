#pragma once
#include "scheduler.hpp"
#include "rocket_sil_framework/include/communication/component_transport.hpp"
#include <map>
#include <set>

namespace component {
// Observation is performed before advancing opaque legacy models: this is the
// snapshot of t, never the already-computed internal state at t+h.
enum class Phase { reveal, compute, prepare, observe, advance };
class PhasedScheduler {
    struct Task {
        double hz;
        Phase phase;
        std::vector<std::string> after;
        std::function<void(double,double)> callback;
    };
    std::map<std::string, Task> tasks_;
    Scheduler scheduler_;
    bool compiled_ = false;
public:
    void addTask(std::string id, double hz, Phase phase, std::vector<std::string> after,
                 std::function<void(double,double)> callback) {
        if (compiled_ || !tasks_.emplace(id, Task{hz,phase,std::move(after),std::move(callback)}).second)
            throw std::invalid_argument("Duplicate or late component task: " + id);
    }
    void compile(double dt) {
        if (compiled_) throw std::logic_error("Scheduler already compiled");
        timeFromSeconds(dt);
        for (const auto& [id, task] : tasks_) {
            if (task.hz > 0) timeFromSeconds(1/task.hz);
            for (const auto& dependency : task.after) {
                auto it = tasks_.find(dependency);
                if (it == tasks_.end() || it->second.phase > task.phase)
                    throw std::invalid_argument(id + ": unknown or future-phase dependency " + dependency);
            }
        }
        std::set<std::string> done;
        uint32_t priority = 0;
        for (auto phase : {Phase::reveal, Phase::compute, Phase::prepare, Phase::observe, Phase::advance}) {
            size_t remaining = 0;
            for (const auto& [id, task] : tasks_) if (task.phase == phase) ++remaining;
            while (remaining) {
                auto it = std::find_if(tasks_.begin(), tasks_.end(), [&](const auto& item) {
                    const auto& [id, task] = item;
                    return task.phase == phase && !done.contains(id) &&
                        std::all_of(task.after.begin(), task.after.end(), [&](const auto& dependency) { return done.contains(dependency); });
                });
                if (it == tasks_.end()) throw std::invalid_argument("Feedthrough cycle in component graph");
                scheduler_.addTask(it->first, it->second.hz, priority++, it->second.callback);
                done.insert(it->first); --remaining;
            }
        }
        scheduler_.validate(dt); compiled_ = true;
    }
    void update(double time, double dt) {
        if (!compiled_) throw std::logic_error("Compile component scheduler first");
        scheduler_.update(time, dt);
    }
};
}
