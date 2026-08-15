#pragma once

#include <vector>
#include <string>

struct ThrustPoint {
    double time_s;
    double thrust_n;
};

class ThrustCurve {
public:
    ThrustCurve() : total_impulse_ns_(0.0), max_burn_time_s_(0.0) {}

    // Load curve from either .csv or .eng file depending on the extension
    bool loadFromFile(const std::string& filepath);
    
    // Get thrust at a specific time (uses linear interpolation)
    double getThrustAt(double time_s) const;
    
    double getTotalImpulse() const { return total_impulse_ns_; }
    double getMaxBurnTime() const { return max_burn_time_s_; }

    // Manually add points (used for constant profile mapping or testing)
    void addPoint(double time_s, double thrust_n);
    void calculateProperties();

private:
    bool parseCsv(const std::string& filepath);
    bool parseEng(const std::string& filepath);

    std::vector<ThrustPoint> points_;
    double total_impulse_ns_;
    double max_burn_time_s_;
};
