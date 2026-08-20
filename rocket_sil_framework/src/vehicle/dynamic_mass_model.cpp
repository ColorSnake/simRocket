#include "rocket_sil_framework/include/vehicle/dynamic_mass_model.hpp"
#include <algorithm>
#include <numeric>

DynamicMassModel::DynamicMassModel(double dry_mass_kg, double dry_cg_z, const Eigen::Vector3d& dry_inertia, std::vector<TankConfig> tanks)
    : dry_mass_kg_(dry_mass_kg), dry_cg_z_(dry_cg_z), dry_inertia_(dry_inertia), tanks_(std::move(tanks)) {
}

MassProperties DynamicMassModel::getProperties() const {
    MassProperties props;
    double total_mass = dry_mass_kg_;
    double sum_moments = dry_mass_kg_ * dry_cg_z_;

    for (const auto& tank : tanks_) {
        if (tank.mass_kg <= 0.0) continue;
        total_mass += tank.mass_kg;
        
        // Objętość cieczy
        double volume = tank.mass_kg / tank.propellant_density_kg_m3;
        double area = M_PI * tank.radius_m * tank.radius_m;
        double current_height = volume / area;
        current_height = std::min(current_height, tank.max_height_m); // Zabezpieczenie
        
        // Środek ciężkości cieczy w zbiorniku
        double cg_liquid = tank.z_bottom_m + current_height / 2.0;
        sum_moments += tank.mass_kg * cg_liquid;
    }

    props.total_mass = total_mass;
    props.center_of_gravity = Eigen::Vector3d(0.0, 0.0, sum_moments / total_mass);
    
    // Obliczanie tensora bezwładności używając twierdzenia Steinera
    double inertia_xx = dry_inertia_.x() + dry_mass_kg_ * std::pow(dry_cg_z_ - props.center_of_gravity.z(), 2);
    double inertia_yy = dry_inertia_.y() + dry_mass_kg_ * std::pow(dry_cg_z_ - props.center_of_gravity.z(), 2);
    double inertia_zz = dry_inertia_.z(); // Z-axis distance is 0 for dry mass relative to its own axis

    for (const auto& tank : tanks_) {
        if (tank.mass_kg <= 0.0) continue;
        
        double volume = tank.mass_kg / tank.propellant_density_kg_m3;
        double area = M_PI * tank.radius_m * tank.radius_m;
        double current_height = std::min(volume / area, tank.max_height_m);
        double cg_liquid = tank.z_bottom_m + current_height / 2.0;
        
        // Lokalny tensor walca cieczy
        double local_ixx = tank.mass_kg * (3.0 * tank.radius_m * tank.radius_m + current_height * current_height) / 12.0;
        double local_izz = tank.mass_kg * (tank.radius_m * tank.radius_m) / 2.0;
        
        // Twierdzenie Steinera (przesunięcie do globalnego CG rakiety)
        double dist_z = cg_liquid - props.center_of_gravity.z();
        double steiner_term = tank.mass_kg * (dist_z * dist_z);
        
        inertia_xx += local_ixx + steiner_term;
        inertia_yy += local_ixx + steiner_term; // Cylinder is symmetric
        inertia_zz += local_izz; // No distance offset on X/Y axes
    }
    
    props.inertia_tensor = Eigen::Vector3d(inertia_xx, inertia_yy, inertia_zz).asDiagonal();

    return props;
}

void DynamicMassModel::update(double mass_flow_rate_kg_s, double dt) {
    if (mass_flow_rate_kg_s <= 0.0) return;

    double mass_to_drain = mass_flow_rate_kg_s * dt;
    double total_propellant = 0.0;
    for (const auto& tank : tanks_) {
        total_propellant += tank.mass_kg;
    }

    if (total_propellant <= 0.0) return;

    // Drenaż proporcjonalny do aktualnej masy zbiornika (zakłada równomierne zużycie O/F)
    for (auto& tank : tanks_) {
        double ratio = tank.mass_kg / total_propellant;
        double drain = mass_to_drain * ratio;
        tank.mass_kg = std::max(0.0, tank.mass_kg - drain);
    }
}
