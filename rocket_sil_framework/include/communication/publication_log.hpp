#pragma once
#include "component_transport.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

namespace component {
// Single-threaded JSONL publication trace, separate from periodic telemetry.
class PublicationLog {
    std::ofstream stream_;
public:
    explicit PublicationLog(const std::string& path);
    void write(const std::string& transport, const Envelope& envelope, const nlohmann::json& payload);
    void finish();
    template<class T, class Encoder> void attach(Output<T>& output, std::string transport, Encoder encoder) {
        output.observePublications([this, transport=std::move(transport), encoder](const Sample<T>& sample) {
            write(transport, sample.envelope, encoder(sample.value));
        });
    }
};
}
