#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include <cmath>

SimpleEnvironmentModel::SimpleEnvironmentModel(const Eigen::Vector3d& gravity, const Eigen::Vector3d& wind_velocity)
    : gravity_(gravity), constant_wind_(wind_velocity) {
}

EnvironmentState SimpleEnvironmentModel::compute(const RocketState& state) {
    EnvironmentState env;
    env.gravity_inertial = gravity_;
    env.wind_velocity_inertial = constant_wind_;
    
    // In our coordinate system, Z=0 is launchpad, and negative Z is down.
    // So altitude is -Z (if Z is positive downwards) or just Z if Z is upwards.
    // Wait, earlier we established nose is Z=0 and tail is Z=-2.
    // In World frame (inertial), Z is usually Up or Down.
    // The user's gravity is -9.81 on Z, which implies Z is UP in inertial frame.
    // Let's assume altitude is state.position.z().
    double altitude = state.position.z();
    if (altitude < 0.0) altitude = 0.0; // Prevent negative altitudes
    
    env.air_density = calculateISADensity(altitude);
    
    // Prosty model ISA dla ciśnienia do ~11km
    env.ambient_pressure_pa = 101325.0 * std::pow(1.0 - 2.25577e-5 * altitude, 5.25588);
    
    return env;
}

double SimpleEnvironmentModel::calculateISADensity(double altitude_m) const {
    // Standard ISA parameters for Troposphere (0 to ~11000m)
    const double p0 = 101325.0; // Sea level standard atmospheric pressure, Pa
    const double T0 = 288.15;   // Sea level standard temperature, K
    const double g = 9.80665;   // Earth-surface gravitational acceleration, m/s^2
    const double L = 0.0065;    // Temperature lapse rate, K/m
    const double R = 287.05;    // Specific gas constant for dry air, J/(kg·K)
    
    if (altitude_m > 11000.0) {
        // Simplified fallback for higher altitudes (constant temp stratosphere)
        altitude_m = 11000.0;
    }
    
    double T = T0 - L * altitude_m;
    double p = p0 * std::pow(1.0 - (L * altitude_m) / T0, (g / (R * L)));
    double density = p / (R * T);
    
    return density;
}
