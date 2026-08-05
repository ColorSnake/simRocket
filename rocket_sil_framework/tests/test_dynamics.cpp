#include <gtest/gtest.h>
#include <memory>
#include "rocket_sil_framework/include/physics/rocket_state.hpp"
#include "rocket_sil_framework/include/physics/rocket_inputs.hpp"
#include "rocket_sil_framework/include/physics/euler_integrator.hpp"
#include "rocket_sil_framework/include/physics/rk4_integrator.hpp"
#include "rocket_sil_framework/include/physics/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/physics/solid_motor_model.hpp"
#include "rocket_sil_framework/include/physics/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

class DynamicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        inputs.gravity_inertial = Eigen::Vector3d(0.0, 0.0, -9.81);
        inputs.force_body = Eigen::Vector3d::Zero();
        inputs.torque_body = Eigen::Vector3d::Zero();
    }

    RocketInputs inputs;
    double dt = 0.001; // 1000 Hz
};

TEST_F(DynamicsTest, FreefallEuler) {
    auto integrator = std::make_unique<EulerIntegrator>();
    auto engine_model = std::make_unique<SolidMotorModel>(0.0, 0.0, 0.0, -2.0);
    auto mass_model = std::make_unique<RigidBodyMassModel>(100.0, 0.0, Eigen::Vector3d(10, 10, 2), -1.0, -1.0);
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(0.0, 0.0, 1.0, -1.5, 0.0, 0.0);
    
    NativeRocketDynamicsModel model(std::move(integrator), std::move(engine_model), std::move(mass_model), std::move(aero_model));
    RocketState state;

    int steps = 4000;
    for (int i = 0; i < steps; ++i) {
        model.update(dt, inputs, state);
    }

    double analytical_z = 0.5 * (-9.81) * 4.0 * 4.0;
    // Euler will have some error, but within 0.1m is expected
    EXPECT_NEAR(state.position.z(), analytical_z, 0.1); 
}

TEST_F(DynamicsTest, FreefallRK4) {
    auto integrator = std::make_unique<RK4Integrator>();
    auto engine_model = std::make_unique<SolidMotorModel>(0.0, 0.0, 0.0, -2.0);
    auto mass_model = std::make_unique<RigidBodyMassModel>(100.0, 0.0, Eigen::Vector3d(10, 10, 2), -1.0, -1.0);
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(0.0, 0.0, 1.0, -1.5, 0.0, 0.0);
    
    NativeRocketDynamicsModel model(std::move(integrator), std::move(engine_model), std::move(mass_model), std::move(aero_model));
    RocketState state;

    int steps = 4000;
    for (int i = 0; i < steps; ++i) {
        model.update(dt, inputs, state);
    }

    double analytical_z = 0.5 * (-9.81) * 4.0 * 4.0;
    // RK4 for constant acceleration should be extremely accurate
    EXPECT_NEAR(state.position.z(), analytical_z, 0.001); 
}

TEST(ConfigTest, LoadPhysicsConfig) {
    std::ifstream config_file("../config.json");
    ASSERT_TRUE(config_file.is_open()) << "Nie udalo sie otworzyc pliku ../config.json!";
    
    nlohmann::json config;
    config_file >> config;

    ASSERT_TRUE(config.contains("rocket"));
    ASSERT_TRUE(config["rocket"].contains("mass"));
    
    double dry_mass = config["rocket"]["mass"]["dry_mass_kg"];
    EXPECT_GT(dry_mass, 0.0) << "Masa sucha musi byc wieksza od 0";
    
    double thrust = config["rocket"]["engine"]["thrust_n"];
    EXPECT_GE(thrust, 0.0) << "Ciag nie moze byc ujemny";
}
