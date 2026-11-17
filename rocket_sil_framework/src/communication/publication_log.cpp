#include "rocket_sil_framework/include/communication/publication_log.hpp"
#include <filesystem>

namespace component {
PublicationLog::PublicationLog(const std::string& path) {
    const std::filesystem::path file(path);
    if (file.empty()) throw std::invalid_argument("Empty bus log path");
    if (std::filesystem::exists(file)) throw std::runtime_error("Bus log already exists: " + path);
    if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path());
    stream_.open(file);
    if (!stream_) throw std::runtime_error("Cannot open bus log: " + path);
}
void PublicationLog::write(const std::string& transport, const Envelope& e, const nlohmann::json& payload) {
    nlohmann::json row = {{"schema", "simrocket/publication-v1"}, {"transport", transport},
        {"source", e.source}, {"sequence", e.sequence}, {"sample_time_ns", e.sample_time},
        {"available_time_ns", e.available_time},
        {"valid_until_ns", e.valid_until == unbounded ? nlohmann::json(nullptr) : nlohmann::json(e.valid_until)},
        {"quality", e.quality == Quality::valid ? "valid" : e.quality == Quality::invalid ? "invalid" : "uninitialized"},
        {"effective_time_ns", e.effective_time ? nlohmann::json(*e.effective_time) : nlohmann::json(nullptr)},
        {"payload", payload}};
    if (!(stream_ << row.dump() << '\n')) throw std::runtime_error("Bus log write failed");
}
void PublicationLog::finish() {
    stream_.flush();
    if (!stream_) throw std::runtime_error("Bus log flush failed");
}
}
