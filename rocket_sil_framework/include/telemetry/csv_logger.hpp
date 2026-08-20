#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <fstream>
#include <mutex>
#include <vector>

class CsvLogger {
public:
    CsvLogger(const std::string& filename);
    ~CsvLogger();

    // Logs the main Telemetry data.
    void log(const nlohmann::json& packet);

private:
    std::ofstream file_;
    std::mutex mutex_;
    bool is_open_;
};
