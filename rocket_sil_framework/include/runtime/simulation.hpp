#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace simrocket {
struct RunOptions {
    std::string config_path = "config.json";
    std::string bus_log_path;
    bool log_csv = false;
};
// Builds the supported preset (or legacy composition), owns its models and
// subscriptions for the complete run, and releases them after the loop exits.
int runConfiguredSystem(nlohmann::json config, const RunOptions& options);
}
