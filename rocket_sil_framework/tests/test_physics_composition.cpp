#include <gtest/gtest.h>
#include "rocket_sil_framework/include/core/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/core/rk4_integrator.hpp"
#include "rocket_sil_framework/include/physics/native_physics_system.hpp"
#include "rocket_sil_framework/include/vehicle/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"

namespace {
class StubPhysics : public IDynamicsPhysics {
public:
    mutable std::vector<double> evaluation_times;
    int prepares=0, commits=0;
    PhysicsStep prepareStep(double dt,const RocketState&,const RocketInputs&) override { ++prepares;return {dt,0}; }
    void advanceActuators(double) override {}
    void consumePropellant(double,double) override { ++commits; }
    DynamicsEvaluation evaluate(const RocketState& state,const RocketInputs&) const override {
        evaluation_times.push_back(state.time);
        DynamicsEvaluation result;
        result.mass=1;result.inertia.setIdentity();result.inertia_inv.setIdentity();
        result.force_body.setZero();result.moment_at_cg_body.setZero();result.gravity_world={0,0,-10};
        return result;
    }
};
class ConstantEngine : public IEngineModel {
public:
    EngineOutput compute(double,const MassProperties&,double pressure) override {
        return {Eigen::Vector3d(0,0,pressure/1000),Eigen::Vector3d::Zero(),2};
    }
    uint32_t getEngineId() const override { return 7; }
};
}
TEST(PhysicsComposition, DynamicsUsesInjectedPhysicsAtEveryRkStage) {
    auto physics=std::make_unique<StubPhysics>();auto* view=physics.get();
    NativeRocketDynamicsModel model(std::make_unique<RK4Integrator>(),std::move(physics));
    RocketState state;RocketInputs inputs;
    model.update(.1,inputs,state);
    EXPECT_NEAR(state.position.z(),-.05,1e-12);
    EXPECT_NEAR(state.velocity.z(),-1,1e-12);
    EXPECT_EQ(view->prepares,1);EXPECT_EQ(view->commits,1);
    ASSERT_EQ(view->evaluation_times.size(),5u);
    EXPECT_NEAR(view->evaluation_times[1],.05,1e-12);
    EXPECT_NEAR(view->evaluation_times.back(),.1,1e-12);
}
TEST(PhysicsComposition, PropulsionConsumesExplicitPressureCgAndTransform) {
    std::vector<std::unique_ptr<IEngineModel>> engines;engines.push_back(std::make_unique<ConstantEngine>());
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    actuators.push_back(std::make_unique<TvcActuatorModel>(3,7,Eigen::Vector3d(1,0,-2),nullptr));
    PropulsionSystem propulsion(std::move(engines),std::move(actuators));
    propulsion.setExternalActuatorScheduling(true);
    propulsion.setActuatorTransform(3,Transform3D({1,0,-2},Eigen::Quaterniond::Identity()));
    MassProperties mass;mass.total_mass=2;mass.propellant_mass=1;mass.center_of_gravity={0,0,-1};mass.inertia_tensor.setIdentity();
    Eigen::Vector3d force=Eigen::Vector3d::Zero(),moment=force;RocketDiagnostics diagnostics;
    propulsion.accumulate(0,mass,10000,force,moment,diagnostics);
    EXPECT_DOUBLE_EQ(force.z(),10);
    EXPECT_DOUBLE_EQ(moment.y(),-10);
    EXPECT_DOUBLE_EQ(propulsion.massFlow(0,mass,10000),2);
    EXPECT_DOUBLE_EQ(diagnostics.engine_thrust_body.at(7).z(),10);
}
TEST(PhysicsComposition, EvaluationDoesNotConsumeFuelAndStepSplitsAtExhaustion) {
    std::vector<std::unique_ptr<IEngineModel>> engines;engines.push_back(std::make_unique<ConstantEngine>());
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    actuators.push_back(std::make_unique<TvcActuatorModel>(3,7,Eigen::Vector3d::Zero(),nullptr));
    auto propulsion=std::make_unique<PropulsionSystem>(std::move(engines),std::move(actuators));
    auto mass=std::make_unique<RigidBodyMassModel>(10,1,Eigen::Vector3d::Ones(),0,0);
    auto* mass_view=mass.get();
    NativePhysicsSystem physics(std::move(propulsion),std::move(mass),nullptr,nullptr);
    RocketState state;RocketInputs inputs;
    const auto step=physics.prepareStep(1,state,inputs);
    EXPECT_DOUBLE_EQ(step.powered_duration,.5);
    for(int i=0;i<4;++i) EXPECT_DOUBLE_EQ(physics.evaluate(state,inputs).mass,11);
    EXPECT_DOUBLE_EQ(mass_view->getProperties().propellant_mass,1);
    physics.consumePropellant(step.mass_flow_kg_s,step.powered_duration);
    EXPECT_DOUBLE_EQ(mass_view->getProperties().propellant_mass,0);
}
