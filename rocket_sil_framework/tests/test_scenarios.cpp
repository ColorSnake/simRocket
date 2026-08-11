#include <gtest/gtest.h>
#include <memory>
#include "rocket_sil_framework/include/physics/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/rk4_integrator.hpp"
#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include "rocket_sil_framework/include/control/tvc_controller.hpp"

class ScenariosTest : public ::testing::Test {
protected:
    void SetUp() override {}

    double dt = 0.01; // 100 Hz for faster tests
};

TEST_F(ScenariosTest, WeathervaneNoTVC) {
    auto env = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d(10.0, 0.0, 0.0)); // 10m/s crosswind
    auto aero = std::make_unique<SimpleAerodynamicsModel>(0.5, 1.0, 0.05, -1.0, 0.1, 0.1); // Cn=1.0, COP=-1.0m
    auto mass = std::make_unique<RigidBodyMassModel>(20.0, 5.0, Eigen::Vector3d(10, 10, 0.1), 0.0, -0.5); // CG at 0.0
    auto motor = std::make_unique<SolidMotorModel>(5.0, 1000.0, 5.0, -2.0, nullptr); // Thrust 1000N, no bus
    auto integrator = std::make_unique<RK4Integrator>();
    
    NativeRocketDynamicsModel rocket(std::move(integrator), std::move(motor), std::move(mass), std::move(aero), std::move(env));
    RocketState state;
    state.position.setZero();
    state.velocity.setZero();
    state.orientation.setIdentity();
    state.angular_velocity.setZero();
    
    RocketInputs inputs;
    inputs.gravity_inertial = Eigen::Vector3d(0, 0, -9.81);
    inputs.force_body.setZero();
    inputs.torque_body.setZero();
    
    // Simulate 2 seconds
    for (int i = 0; i < 200; ++i) {
        rocket.update(dt, inputs, state);
    }
    
    // Wind is +X (East). The rocket is moving up (+Z). 
    // Relative wind hits from +X side. It should rotate pitch around -Y to point INTO the wind.
    // So the z_axis.x() should become negative.
    Eigen::Vector3d z_axis = state.orientation * Eigen::Vector3d::UnitZ();
    EXPECT_LT(z_axis.x(), -0.1); // Rocket is leaning into the wind
}

TEST_F(ScenariosTest, StabilizedTVC) {
    auto bus = std::make_shared<MessageBus>();
    TvcController tvc(bus, 0.1, 0.05, 0.1); // Lower PID gains for test stability
    
    auto env = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d(10.0, 0.0, 0.0));
    auto aero = std::make_unique<SimpleAerodynamicsModel>(0.5, 1.0, 0.05, -1.0, 0.1, 0.1);
    auto mass = std::make_unique<RigidBodyMassModel>(20.0, 5.0, Eigen::Vector3d(10, 10, 0.1), 0.0, -0.5);
    auto motor = std::make_unique<SolidMotorModel>(5.0, 1000.0, 5.0, -2.0, bus);
    auto integrator = std::make_unique<RK4Integrator>();
    
    NativeRocketDynamicsModel rocket(std::move(integrator), std::move(motor), std::move(mass), std::move(aero), std::move(env));
    RocketState state;
    state.position.setZero();
    state.velocity.setZero();
    state.orientation.setIdentity();
    state.angular_velocity.setZero();
    
    RocketInputs inputs;
    inputs.gravity_inertial = Eigen::Vector3d(0, 0, -9.81);
    inputs.force_body.setZero();
    inputs.torque_body.setZero();
    
    // Simulate 2 seconds
    for (int i = 0; i < 200; ++i) {
        ImuStateMessage imu;
        imu.orientation = state.orientation;
        imu.angular_velocity = state.angular_velocity;
        bus->publish(imu);
        
        tvc.update(dt);
        rocket.update(dt, inputs, state);
    }
    
    // TVC should fight the wind and keep it much closer to vertical
    Eigen::Vector3d z_axis = state.orientation * Eigen::Vector3d::UnitZ();
    EXPECT_LT(std::abs(z_axis.x()), 0.1); // Kept within a tiny margin
}
