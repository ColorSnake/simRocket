#include "rocket_sil_framework/include/config/scenario_loader.hpp"
#include "rocket_sil_framework/include/runtime/simulation.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

int run(int argc, char* argv[]) {
    // CLI and configuration loading; model composition belongs to runtime.
    std::string config_path = "config.json";
    std::string bus_log_path;
    bool log_csv = false, validate_only = false, dump_config = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--log-csv") {
            log_csv = true;
        } else if (arg == "--log-bus") {
            if (++i >= argc || std::string(argv[i]).starts_with("--"))
                throw std::invalid_argument("--log-bus requires an output JSONL path");
            bus_log_path = argv[i];
            if (bus_log_path.empty()) throw std::invalid_argument("Empty --log-bus path");
        } else if (arg == "--validate") {
            validate_only = true;
        } else if (arg == "--dump-config") {
            dump_config = true;
        } else if (!arg.empty() && arg[0] != '-') {
            config_path = arg;
        } else { throw std::invalid_argument("Unknown argument: " + arg); }
    }

    nlohmann::json config = loadScenario(config_path);
    const bool component_mode = config.contains("component_runtime");
    if (!bus_log_path.empty() && !component_mode)
        throw std::invalid_argument("--log-bus requires a component scenario (YAML or resolved JSON)");
    if (validate_only || dump_config) {
        if (!component_mode) throw std::invalid_argument("--validate/--dump-config currently support component YAML only");
        if (dump_config) std::cout << config.dump(2) << '\n';
        else std::cout << "Configuration valid: " << config_path << '\n';
        return 0;
    }
    return simrocket::runConfiguredSystem(std::move(config), {config_path, bus_log_path, log_csv});
}

int main(int argc, char* argv[]) {
    try { return run(argc, argv); }
    catch (const std::exception& error) {
        std::cerr << "[simRocket] " << error.what() << '\n';
        return 1;
    }
}
