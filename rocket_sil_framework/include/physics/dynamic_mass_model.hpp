#pragma once

#include "i_mass_model.hpp"
#include <vector>

struct TankConfig {
    double z_bottom_m;              // Position of the tank bottom on the Z-axis
    double radius_m;                // Inner radius of the tank
    double max_height_m;            // Max physical height of the tank
    double propellant_density_kg_m3;// Density of the propellant (e.g., LOX ~1141, Kerosene ~820)
    double mass_kg;                 // Current propellant mass
};

class DynamicMassModel : public IMassModel {
public:
    // dry_mass_kg: Masa samej struktury bez paliwa
    // dry_cg_z: Środek ciężkości (Z) dla pustej rakiety
    // dry_inertia: Wektor bezwładności pustej rakiety (Ixx, Iyy, Izz)
    // tanks: Lista zbiorników z paliwem
    DynamicMassModel(double dry_mass_kg, double dry_cg_z, const Eigen::Vector3d& dry_inertia, std::vector<TankConfig> tanks);
    ~DynamicMassModel() override = default;

    MassProperties getProperties() const override;
    void update(double mass_flow_rate_kg_s, double dt) override;

private:
    double dry_mass_kg_;
    double dry_cg_z_;
    Eigen::Vector3d dry_inertia_;
    std::vector<TankConfig> tanks_;
};
