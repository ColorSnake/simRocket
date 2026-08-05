#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include <algorithm>

RigidBodyMassModel::RigidBodyMassModel(double dry_mass_kg, double initial_propellant_mass_kg, 
                                       const Eigen::Vector3d& dry_inertia_diagonal,
                                       double dry_cg_z, double propellant_cg_z)
    : dry_mass_kg_(dry_mass_kg), 
      current_propellant_mass_kg_(initial_propellant_mass_kg),
      initial_propellant_mass_kg_(initial_propellant_mass_kg),
      dry_cg_z_(dry_cg_z),
      propellant_cg_z_(propellant_cg_z),
      dry_inertia_diagonal_(dry_inertia_diagonal) {
    
    center_of_gravity_.setZero();
    inertia_tensor_.setIdentity();
}

void RigidBodyMassModel::update(double mass_flow_rate, double dt) {
    double mass_burned = mass_flow_rate * dt;
    current_propellant_mass_kg_ -= mass_burned;
    
    // Zabezpieczenie przed ujemną masą paliwa
    if (current_propellant_mass_kg_ < 0.0) {
        current_propellant_mass_kg_ = 0.0;
    }
}

MassProperties RigidBodyMassModel::getProperties() const {
    MassProperties props;
    props.total_mass = dry_mass_kg_ + current_propellant_mass_kg_;
    
    // Obliczenie wypadkowego Środka Ciężkości (CG) na osi Z
    double cg_z = (dry_mass_kg_ * dry_cg_z_ + current_propellant_mass_kg_ * propellant_cg_z_) / props.total_mass;
    props.center_of_gravity = Eigen::Vector3d(0.0, 0.0, cg_z);
    
    // Aproksymacja spadku momentu bezwładności proporcjonalnie do ubytku masy.
    // Używamy ułamka całkowitej masy obecnej do startowej.
    // To duże uproszczenie, ale zachowuje fizyczny trend spadku bezwładności.
    double mass_ratio = props.total_mass / (dry_mass_kg_ + initial_propellant_mass_kg_);
    Eigen::Vector3d current_inertia_diagonal = dry_inertia_diagonal_ * mass_ratio;
    
    props.inertia_tensor = Eigen::Matrix3d::Identity();
    props.inertia_tensor(0, 0) = current_inertia_diagonal.x();
    props.inertia_tensor(1, 1) = current_inertia_diagonal.y();
    props.inertia_tensor(2, 2) = current_inertia_diagonal.z();
    
    return props;
}
