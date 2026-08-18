#pragma once

#include "rocket_sil_framework/include/telemetry_packet.hpp"
#include <string>
#include <fstream>
#include <mutex>
#include <vector>

class CsvLogger {
public:
    CsvLogger(const std::string& filename);
    ~CsvLogger();

    // Logs the main TelemetryPacket data.
    // Engine data is currently not logged to keep the CSV simple and flat,
    // but can be added if needed.
    void log(const TelemetryPacket& packet);

private:
    std::ofstream file_;
    std::mutex mutex_;
    bool is_open_;
};
