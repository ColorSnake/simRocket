#pragma once

#include "i_mass_model.hpp"

class RigidBodyMassModel : public IMassModel {
public:
    RigidBodyMassModel(double dry_mass_kg, double initial_propellant_mass_kg, 
                       const Eigen::Vector3d& dry_inertia_diagonal,
                       double dry_cg_z, double propellant_cg_z);
    
    void update(double mass_flow_rate, double dt) override;
    MassProperties getProperties() const override;

private:
    double dry_mass_kg_;
    double current_propellant_mass_kg_;
    double initial_propellant_mass_kg_;
    
    double dry_cg_z_;
    double propellant_cg_z_;
    
    Eigen::Vector3d dry_inertia_diagonal_;
    
    Eigen::Vector3d center_of_gravity_;
    Eigen::Matrix3d inertia_tensor_;
};
