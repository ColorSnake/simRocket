#pragma once
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

// Scalar Real signals in SI. Names include frame and physical meaning.
class SignalRegistry {
public:
    struct Sample { double value = 0; double time = 0; std::string unit; };
    void declare(const std::string& name, const std::string& unit, double initial = 0) {
        if (name.empty() || !std::isfinite(initial) || !signals_.emplace(name, Sample{initial, 0, unit}).second)
            throw std::invalid_argument("Invalid or duplicate signal: " + name);
    }
    const Sample& get(const std::string& name) const {
        auto it = signals_.find(name);
        if (it == signals_.end()) throw std::invalid_argument("Unknown signal: " + name);
        return it->second;
    }
    void set(const std::string& name, double value, double time) {
        get(name);
        if (!std::isfinite(value) || !std::isfinite(time)) throw std::invalid_argument("Non-finite signal: " + name);
        signals_.at(name).value = value; signals_.at(name).time = time;
    }
    const auto& samples() const { return signals_; }
private:
    std::map<std::string, Sample> signals_;
};
