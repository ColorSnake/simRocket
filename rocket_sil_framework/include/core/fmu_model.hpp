#pragma once
#include <map>
#include <string>
#include <vector>
#include "third_party/fmi2/fmi2FunctionTypes.h"

// FMI runtime only: no rocket state, sensor names or MessageBus dependencies.
// Supported contract: FMI 2.0 synchronous Co-Simulation, scalar Real ports.
class FmuModel {
public:
    struct Variable {
        fmi2ValueReference reference;
        std::string causality;
        std::string variability;
        bool real;
        std::string unit;
    };
    FmuModel(const std::string& path, const std::string& name);
    ~FmuModel();
    FmuModel(const FmuModel&) = delete;
    FmuModel& operator=(const FmuModel&) = delete;
    const Variable& variable(const std::string& name) const;
    void initialize(const std::map<std::string, double>& parameters,
                    const std::map<std::string, double>& inputs, double stop_time);
    void setInputs(const std::map<std::string, double>& values);
    double getReal(const std::string& name) const;
    void step(double time, double dt);
private:
    void cleanup() noexcept;
    void check(fmi2Status status, const char* operation) const;
    void set(const std::string& name, double value);
    std::string name_, directory_;
    std::map<std::string, Variable> variables_;
    void* library_ = nullptr;
    fmi2Component component_ = nullptr;
    fmi2CallbackFunctions callbacks_{};
    bool initialized_ = false;
    mutable bool failed_ = false;
    bool variable_step_ = false;
    double time_ = 0, step_size_ = 0;
    fmi2InstantiateTYPE* instantiate_ = nullptr;
    fmi2FreeInstanceTYPE* free_ = nullptr;
    fmi2SetupExperimentTYPE* setup_ = nullptr;
    fmi2EnterInitializationModeTYPE* enter_ = nullptr;
    fmi2ExitInitializationModeTYPE* exit_ = nullptr;
    fmi2SetRealTYPE* set_real_ = nullptr;
    fmi2GetRealTYPE* get_real_ = nullptr;
    fmi2DoStepTYPE* do_step_ = nullptr;
    fmi2TerminateTYPE* terminate_ = nullptr;
};
