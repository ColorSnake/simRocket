#pragma once
#include "fmu_model.hpp"
#include "signal_registry.hpp"
#include <nlohmann/json.hpp>
#include <memory>

// Binding keys are FMI variable names; values are registry signal names.
// A completed step is buffered until its end time: no future output leakage.
class FmuComponent {
public:
    FmuComponent(const std::string& name, const nlohmann::json& config, SignalRegistry& signals)
        : name_(name), config_(config), signals_(signals), model_(config.at("path"), name) {
        inputs_ = config.at("inputs").get<std::map<std::string, std::string>>();
        outputs_ = config.at("outputs").get<std::map<std::string, std::string>>();
        if (outputs_.empty()) throw std::invalid_argument("FMU needs output mappings: " + name);
        for (const auto& [variable, signal] : outputs_) {
            if (model_.variable(variable).causality != "output") throw std::invalid_argument("Not an FMI output: " + variable);
            const auto xml_unit = model_.variable(variable).unit;
            const auto unit = config.value("units", nlohmann::json::object()).value(signal, xml_unit.empty() ? "unspecified" : xml_unit);
            if (!xml_unit.empty() && unit != xml_unit) throw std::invalid_argument("FMU output unit mismatch: " + variable);
            signals_.declare(signal, unit);
        }
    }
    void initialize(double stop_time) {
        for (const auto& [variable, signal] : inputs_) {
            if (model_.variable(variable).causality != "input") throw std::invalid_argument("Not an FMI input: " + variable);
            const auto sample = signals_.get(signal);
            const auto unit = model_.variable(variable).unit;
            if (!unit.empty() && sample.unit != "unspecified" && unit != sample.unit)
                throw std::invalid_argument("FMU input unit mismatch: " + variable);
        }
        model_.initialize(config_.value("parameters", std::map<std::string, double>{}), readInputs(0), stop_time);
        capture(0, 0);
    }
    bool publishReady(double time) {
        if (!pending_ || time + 1e-9 < available_at_) return false;
        pending_ = false;
        if (config_.value("discard_repeated_samples", false) && sample_at_ <= published_sample_time_) return false;
        for (const auto& [signal, value] : pending_values_) signals_.set(signal, value, sample_at_);
        published_sample_time_ = sample_at_;
        return true;
    }
    void step(double time, double dt) {
        if (pending_) throw std::logic_error("FMU outputs must be published before next step");
        model_.setInputs(readInputs(time)); model_.step(time, dt);
        double sample_time = time + dt;
        if (config_.value("sample_time_policy", std::string("step_end")) == "latched_input") {
            if (inputs_.empty()) throw std::invalid_argument("latched_input requires input bindings");
            sample_time = signals_.get(inputs_.begin()->second).time;
            for (const auto& [variable, signal] : inputs_)
                if (std::abs(signals_.get(signal).time - sample_time) > 1e-9)
                    throw std::invalid_argument("FMU input axes must share a sample time");
        }
        capture(time + dt, sample_time);
    }
    double sampleTime() const { return sample_at_; }
    double availableTime() const { return available_at_; }
    const std::string& name() const { return name_; }
    const nlohmann::json& config() const { return config_; }
private:
    std::map<std::string, double> readInputs(double time) {
        std::map<std::string, double> values;
        for (const auto& [variable, signal] : inputs_) {
            auto sample = signals_.get(signal);
            if (sample.time > time + 1e-9) throw std::logic_error("Future input signal: " + signal);
            values[variable] = sample.value;
        }
        return values;
    }
    void capture(double time, double sample_time) {
        pending_values_.clear();
        for (const auto& [variable, signal] : outputs_) pending_values_[signal] = model_.getReal(variable);
        available_at_ = time; sample_at_ = sample_time; pending_ = true;
    }
    std::string name_;
    nlohmann::json config_;
    SignalRegistry& signals_;
    FmuModel model_;
    std::map<std::string, std::string> inputs_, outputs_;
    std::map<std::string, double> pending_values_;
    double available_at_ = 0;
    double sample_at_ = 0;
    double published_sample_time_ = -1;
    bool pending_ = false;
};
