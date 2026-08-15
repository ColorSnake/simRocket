#include "rocket_sil_framework/include/physics/thrust_curve.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

bool ThrustCurve::loadFromFile(const std::string& filepath) {
    points_.clear();
    bool success = false;
    
    if (filepath.length() >= 4 && filepath.substr(filepath.length() - 4) == ".eng") {
        success = parseEng(filepath);
    } else {
        success = parseCsv(filepath);
    }

    if (success) {
        calculateProperties();
    }
    return success;
}

void ThrustCurve::addPoint(double time_s, double thrust_n) {
    points_.push_back({time_s, thrust_n});
}

void ThrustCurve::calculateProperties() {
    if (points_.empty()) {
        total_impulse_ns_ = 0.0;
        max_burn_time_s_ = 0.0;
        return;
    }

    std::sort(points_.begin(), points_.end(), [](const ThrustPoint& a, const ThrustPoint& b) {
        return a.time_s < b.time_s;
    });

    max_burn_time_s_ = points_.back().time_s;
    
    // Calculate total impulse using trapezoidal integration
    total_impulse_ns_ = 0.0;
    for (size_t i = 1; i < points_.size(); ++i) {
        double dt = points_[i].time_s - points_[i-1].time_s;
        double avg_thrust = (points_[i].thrust_n + points_[i-1].thrust_n) / 2.0;
        total_impulse_ns_ += avg_thrust * dt;
    }
}

double ThrustCurve::getThrustAt(double time_s) const {
    if (points_.empty() || time_s < points_.front().time_s || time_s > points_.back().time_s) {
        return 0.0;
    }

    // Find the right interval
    for (size_t i = 1; i < points_.size(); ++i) {
        if (time_s <= points_[i].time_s) {
            double t0 = points_[i-1].time_s;
            double t1 = points_[i].time_s;
            double f0 = points_[i-1].thrust_n;
            double f1 = points_[i].thrust_n;

            if (t1 == t0) return f0; // avoid division by zero
            
            // Linear interpolation
            double ratio = (time_s - t0) / (t1 - t0);
            return f0 + ratio * (f1 - f0);
        }
    }
    return 0.0;
}

bool ThrustCurve::parseCsv(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string time_str, thrust_str;
        
        if (std::getline(ss, time_str, ',') && std::getline(ss, thrust_str, ',')) {
            try {
                double t = std::stod(time_str);
                double f = std::stod(thrust_str);
                points_.push_back({t, f});
            } catch(...) {
                // skip invalid lines
            }
        }
    }
    return !points_.empty();
}

bool ThrustCurve::parseEng(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::string line;
    bool header_found = false;

    while (std::getline(file, line)) {
        // Trim leading whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        if (line.empty() || line[0] == ';') continue;

        if (!header_found) {
            // First non-comment line is the header, skip parsing it for now
            header_found = true;
            continue;
        }

        // Data points (Time Thrust)
        std::stringstream ss(line);
        double t, f;
        if (ss >> t >> f) {
            points_.push_back({t, f});
        }
    }
    return !points_.empty();
}
