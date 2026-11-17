#include "rocket_sil_framework/include/core/fmu_model.hpp"
#include <pugixml.hpp>
#include <dlfcn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <regex>
#include <stdexcept>
namespace fs = std::filesystem;
namespace {
void logger(fmi2ComponentEnvironment, fmi2String name, fmi2Status,
            fmi2String, fmi2String format, ...) {
    std::fprintf(stderr, "[FMU:%s] ", name ? name : "?");
    va_list args; va_start(args, format); std::vfprintf(stderr, format, args); va_end(args);
    std::fputc('\n', stderr);
}
}
FmuModel::FmuModel(const std::string& path, const std::string& name) : name_(name) {
    try {
        char pattern[] = "/tmp/simrocket-fmu-XXXXXX";
        char* directory = mkdtemp(pattern);
        if (!directory) throw std::runtime_error("Cannot create FMU temporary directory");
        directory_ = directory;
        const auto absolute = fs::absolute(path).string();
        pid_t child = fork();
        if (child < 0) throw std::runtime_error("Cannot start FMU extractor");
        if (child == 0) {
            execl(SIMROCKET_PYTHON, SIMROCKET_PYTHON, SIMROCKET_EXTRACTOR,
                  absolute.c_str(), directory_.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }
        int status = 0;
        pid_t result;
        do { result = waitpid(child, &status, 0); } while (result < 0 && errno == EINTR);
        if (result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
            throw std::runtime_error("Cannot extract FMU: " + path);
        pugi::xml_document xml;
        if (!xml.load_file((directory_ + "/modelDescription.xml").c_str()))
            throw std::runtime_error("Invalid modelDescription.xml");
        const auto root = xml.child("fmiModelDescription");
        const auto cs = root.child("CoSimulation");
        const std::string identifier = cs.attribute("modelIdentifier").as_string();
        const std::string guid = root.attribute("guid").as_string();
        if (std::string(root.attribute("fmiVersion").as_string()) != "2.0" || !cs || guid.empty()
                || !std::regex_match(identifier, std::regex("[A-Za-z_][A-Za-z0-9_]*")))
            throw std::runtime_error("Expected FMI 2.0 Co-Simulation with valid identifier and GUID");
        if (cs.attribute("canRunAsynchronuously").as_bool())
            throw std::runtime_error("Asynchronous FMUs are not supported");
        variable_step_ = cs.attribute("canHandleVariableCommunicationStepSize").as_bool();
        for (auto node : root.child("ModelVariables").children("ScalarVariable")) {
            std::string variable_name = node.attribute("name").as_string();
            std::string reference = node.attribute("valueReference").as_string();
            if (variable_name.empty() || !std::regex_match(reference, std::regex("[0-9]+"))
                    || std::stoull(reference) > UINT32_MAX)
                throw std::runtime_error("Invalid FMI variable metadata");
            Variable v{static_cast<fmi2ValueReference>(std::stoull(reference)),
                node.attribute("causality").as_string("local"),
                node.attribute("variability").as_string("continuous"), bool(node.child("Real")), node.child("Real").attribute("unit").as_string()};
            if (!variables_.emplace(variable_name, v).second)
                throw std::runtime_error("Duplicate FMI variable: " + variable_name);
        }
        library_ = dlopen((directory_ + "/binaries/linux64/" + identifier + ".so").c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!library_) throw std::runtime_error(std::string("FMU dlopen: ") + dlerror());
#define LOAD(member, symbol) member = reinterpret_cast<decltype(member)>(dlsym(library_, symbol)); if (!member) throw std::runtime_error("Missing FMI symbol: " symbol)
        LOAD(instantiate_, "fmi2Instantiate"); LOAD(free_, "fmi2FreeInstance");
        LOAD(setup_, "fmi2SetupExperiment"); LOAD(enter_, "fmi2EnterInitializationMode");
        LOAD(exit_, "fmi2ExitInitializationMode"); LOAD(set_real_, "fmi2SetReal");
        LOAD(get_real_, "fmi2GetReal"); LOAD(do_step_, "fmi2DoStep"); LOAD(terminate_, "fmi2Terminate");
#undef LOAD
        auto version = reinterpret_cast<fmi2GetVersionTYPE*>(dlsym(library_, "fmi2GetVersion"));
        if (!version || std::string(version()) != "2.0") throw std::runtime_error("FMU binary is not FMI 2.0");
        callbacks_.logger = logger; callbacks_.allocateMemory = calloc; callbacks_.freeMemory = std::free;
        fs::create_directories(directory_ + "/resources");
        const std::string resources = "file://" + directory_ + "/resources/";
        component_ = instantiate_(name_.c_str(), fmi2CoSimulation, guid.c_str(), resources.c_str(), &callbacks_, fmi2False, fmi2False);
        if (!component_) throw std::runtime_error("FMU instantiation failed: " + name_);
    } catch (...) { cleanup(); throw; }
}
FmuModel::~FmuModel() { cleanup(); }
void FmuModel::cleanup() noexcept {
    if (component_) {
        if (initialized_ && !failed_ && terminate_) terminate_(component_);
        if (free_) free_(component_);
        component_ = nullptr;
    }
    if (library_) { dlclose(library_); library_ = nullptr; }
    if (!directory_.empty()) { std::error_code error; fs::remove_all(directory_, error); }
}
void FmuModel::check(fmi2Status status, const char* operation) const {
    if (status == fmi2Warning) std::cerr << "[FMU:" << name_ << "] warning in " << operation << '\n';
    else if (status != fmi2OK) { failed_ = true; throw std::runtime_error("FMU '" + name_ + "': " + operation + " returned status " + std::to_string(status)); }
}
const FmuModel::Variable& FmuModel::variable(const std::string& name) const {
    auto it = variables_.find(name);
    if (it == variables_.end()) throw std::invalid_argument("Unknown FMI variable: " + name);
    if (!it->second.real) throw std::invalid_argument("Only scalar Real mappings are supported: " + name);
    return it->second;
}
void FmuModel::set(const std::string& name, double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("Non-finite FMI value: " + name);
    auto ref = variable(name).reference;
    check(set_real_(component_, &ref, 1, &value), "fmi2SetReal");
}
void FmuModel::initialize(const std::map<std::string, double>& parameters,
                          const std::map<std::string, double>& inputs, double stop_time) {
    if (!std::isfinite(stop_time) || stop_time <= 0) throw std::invalid_argument("Invalid FMI stop time");
    if (initialized_) throw std::logic_error("FMU already initialized");
    try {
        check(setup_(component_, fmi2False, 0, 0, fmi2True, stop_time), "fmi2SetupExperiment");
        for (const auto& [name, value] : parameters) {
            if (variable(name).causality != "parameter") throw std::invalid_argument("Not an FMI parameter: " + name);
            set(name, value);
        }
        setInputs(inputs);
        check(enter_(component_), "fmi2EnterInitializationMode");
        check(exit_(component_), "fmi2ExitInitializationMode");
        initialized_ = true;
    } catch (...) { failed_ = true; throw; }
}
void FmuModel::setInputs(const std::map<std::string, double>& values) {
    for (const auto& [name, value] : values) {
        if (variable(name).causality != "input") throw std::invalid_argument("Not an FMI input: " + name);
        set(name, value);
    }
}
double FmuModel::getReal(const std::string& name) const {
    auto ref = variable(name).reference; double value = 0;
    check(get_real_(component_, &ref, 1, &value), "fmi2GetReal");
    if (!std::isfinite(value)) throw std::runtime_error("Non-finite FMI output: " + name);
    return value;
}
void FmuModel::step(double time, double dt) {
    if (!initialized_ || failed_ || !std::isfinite(time) || !std::isfinite(dt) || dt <= 0 || std::abs(time-time_) > 1e-8)
        throw std::invalid_argument("Invalid FMU communication time: " + name_);
    if (!variable_step_ && step_size_ != 0 && std::abs(dt-step_size_) > 1e-10)
        throw std::invalid_argument("FMU requires constant communication step: " + name_);
    try { check(do_step_(component_, time, dt, fmi2True), "fmi2DoStep"); }
    catch (...) { failed_ = true; throw; }
    time_ += dt; step_size_ = dt;
}
