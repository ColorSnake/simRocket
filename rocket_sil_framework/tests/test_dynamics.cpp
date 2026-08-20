#include <gtest/gtest.h>
#include <memory>
#include "rocket_sil_framework/include/core/rocket_state.hpp"
#include "rocket_sil_framework/include/core/rocket_inputs.hpp"
#include "rocket_sil_framework/include/core/euler_integrator.hpp"
#include "rocket_sil_framework/include/core/rk4_integrator.hpp"
#include "rocket_sil_framework/include/core/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/plant_models/solid_motor_model.hpp"
#include "rocket_sil_framework/include/vehicle/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/physics/simple_aerodynamics_model.hpp"
#include "rocket_sil_framework/include/physics/simple_environment_model.hpp"
#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"
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
    auto curve = std::make_shared<ThrustCurve>();
    std::vector<std::unique_ptr<IEngineModel>> engines;
    engines.push_back(std::make_unique<SolidMotorModel>(0, curve, 0.0));
    
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    auto bus = std::make_shared<MessageBus>();
    actuators.push_back(std::make_unique<TvcActuatorModel>(0, 0, Eigen::Vector3d(0, 0, -2.0), bus));
    
    auto mass_model = std::make_unique<RigidBodyMassModel>(100.0, 0.0, Eigen::Vector3d(10, 10, 2), -1.0, -1.0);
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(0.0, 0.0, 1.0, -1.5, 0.0, 0.0);
    auto env_model = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d(0, 0, 0));
    
    NativeRocketDynamicsModel model(std::move(integrator), std::move(engines), std::move(actuators), std::move(mass_model), std::move(aero_model), std::move(env_model));
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
    auto mass_model = std::make_unique<RigidBodyMassModel>(10.0, 0.0, Eigen::Vector3d::Ones(), 0.0, 0.0);
    
    auto curve = std::make_shared<ThrustCurve>();
    
    std::vector<std::unique_ptr<IEngineModel>> engines;
    engines.push_back(std::make_unique<SolidMotorModel>(0, curve, 0.0));
    
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    auto bus = std::make_shared<MessageBus>();
    actuators.push_back(std::make_unique<TvcActuatorModel>(0, 0, Eigen::Vector3d(0, 0, -2.0), bus));
    
    auto integrator = std::make_unique<RK4Integrator>();
    auto aero_model = std::make_unique<SimpleAerodynamicsModel>(0.0, 0.0, 1.0, -1.5, 0.0, 0.0);
    auto env_model = std::make_unique<SimpleEnvironmentModel>(Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d(0, 0, 0));
    
    NativeRocketDynamicsModel model(std::move(integrator), std::move(engines), std::move(actuators), std::move(mass_model), std::move(aero_model), std::move(env_model));
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
    std::ifstream config_file("config.json");
    if (!config_file.is_open()) {
        config_file.open("../config.json");
    }
    ASSERT_TRUE(config_file.is_open()) << "Nie udalo sie otworzyc pliku config.json!";
    
    nlohmann::json config;
    config_file >> config;

    ASSERT_TRUE(config.contains("rocket"));
    ASSERT_TRUE(config["rocket"].contains("mass"));
    
    double dry_mass = config["rocket"]["mass"]["dry_mass_kg"];
    EXPECT_GT(dry_mass, 0.0) << "Masa sucha musi byc wieksza od 0";
    
    ASSERT_TRUE(config["rocket"].contains("engines"));
    double prop_mass = config["rocket"]["engines"][0]["propellant_mass_kg"];
    EXPECT_GE(prop_mass, 0.0) << "Masa paliwa nie moze byc ujemna";
}
