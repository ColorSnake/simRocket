#include <gtest/gtest.h>
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"

TEST(AerodynamicsTest, DragCalculation) {
    // Parameters
    double Cd = 0.5;
    double Cn = 0.0;
    double ref_area = 1.0;
    double cop_z = -1.0;
    
    SimpleAerodynamicsModel aero(Cd, Cn, ref_area, cop_z, 0.0, 0.0);
    
    RocketState state;
    state.orientation.setIdentity();
    state.velocity = Eigen::Vector3d(0.0, 0.0, 100.0); // Moving up at 100 m/s
    state.angular_velocity.setZero();
    
    MassProperties mass; // unused by simple aero
    
    EnvironmentState env;
    env.air_density = 1.225;
    env.wind_velocity_inertial.setZero();
    
    AeroForces forces = aero.compute(state, mass, env);
    
    // Fd = 0.5 * rho * v^2 * Cd * A
    // Fd = 0.5 * 1.225 * 10000 * 0.5 * 1.0 = 3062.5 N
    // Since moving +Z, drag should be -Z
    EXPECT_NEAR(forces.aerodynamic_force_body.z(), -3062.5, 0.1);
    EXPECT_NEAR(forces.aerodynamic_force_body.x(), 0.0, 0.1);
    EXPECT_NEAR(forces.aerodynamic_force_body.y(), 0.0, 0.1);
}
