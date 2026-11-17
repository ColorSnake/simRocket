#pragma once
#include <nlohmann/json.hpp>
#include <string>
// YAML compilation validates metadata only; it never dlopens an FMU.
nlohmann::json loadScenario(const std::string& path);
