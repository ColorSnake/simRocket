#include "rocket_sil_framework/include/config/scenario_loader.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>

nlohmann::json loadScenario(const std::string& path) {
    const auto suffix = std::filesystem::path(path).extension();
    if (suffix != ".yaml" && suffix != ".yml") {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("Cannot open configuration: " + path);
        return nlohmann::json::parse(input);
    }
    int output[2];
    if (pipe(output) < 0) throw std::runtime_error("Cannot create configuration compiler pipe");
    const auto absolute = std::filesystem::absolute(path).string();
    const pid_t child = fork();
    if (child < 0) { close(output[0]); close(output[1]); throw std::runtime_error("Cannot launch configuration compiler"); }
    if (child == 0) {
        close(output[0]);
        if (dup2(output[1], STDOUT_FILENO) < 0) _exit(127);
        close(output[1]);
        execl(SIMROCKET_PYTHON, SIMROCKET_PYTHON, SIMROCKET_SCENARIO_LOADER, absolute.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    close(output[1]);
    std::string json;
    char buffer[8192];
    bool failed = false;
    while (true) {
        const auto count = read(output[0], buffer, sizeof(buffer));
        if (count < 0 && errno == EINTR) continue;
        if (count < 0) failed = true;
        if (count <= 0) break;
        if (json.size() + count > 16*1024*1024) { failed = true; break; }
        json.append(buffer, count);
    }
    close(output[0]);
    int status = 0; pid_t result;
    do { result = waitpid(child, &status, 0); } while (result < 0 && errno == EINTR);
    if (failed || result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("YAML configuration rejected (see diagnostic above)");
    return nlohmann::json::parse(json);
}
