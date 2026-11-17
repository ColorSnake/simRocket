#include <gtest/gtest.h>
#include "rocket_sil_framework/include/core/scheduler.hpp"
#include "rocket_sil_framework/include/sensors/imu_sensor_model.hpp"
#include "rocket_sil_framework/include/estimation/error_state_kalman_filter.hpp"

TEST(TimeContract, ModuleReceivesOwnPeriod) {
    Scheduler scheduler;
    double total = 0; int count = 0;
    scheduler.addTask("imu", 100, 0, [&](double, double dt) { total += dt; ++count; });
    for (int i=0; i<1000; ++i) scheduler.update(i * .001, .001);
    EXPECT_EQ(count, 100); EXPECT_NEAR(total, 1, 1e-12);
}
TEST(TimeContract, Exact800HzOn4000HzGrid) {
    Scheduler scheduler;
    int count = 0;
    scheduler.addTask("imu", 800, 0, [&](double time, double dt) {
        EXPECT_NEAR(dt, .00125, 1e-12); EXPECT_NEAR(time, count * .00125, 1e-12); ++count;
    });
    EXPECT_THROW(scheduler.validate(.001), std::invalid_argument);
    for (int i=0; i<4000; ++i) scheduler.update(i * .00025, .00025);
    EXPECT_EQ(count, 800);
}
TEST(ImuContract, FreeFallIsZeroAndRestMeasuresGravityInBody) {
    MessageBus bus; ImuSensorModel imu(bus); ImuStateMessage measured;
    bus.subscribe<ImuStateMessage>([&](const auto& msg) { measured=msg; });
    RocketState state;
    state.orientation = Eigen::AngleAxisd(.8, Eigen::Vector3d::UnitY());
    state.acceleration = Eigen::Vector3d(0,0,-9.81);
    imu.update(.01, .1, state);
    EXPECT_NEAR(measured.linear_acceleration.norm(), 0, 1e-12);
    state.acceleration.setZero(); imu.update(.01, .2, state);
    EXPECT_NEAR((state.orientation * measured.linear_acceleration - Eigen::Vector3d(0,0,9.81)).norm(), 0, 1e-12);
    EXPECT_DOUBLE_EQ(measured.timestamp_s, .2);
}
TEST(EstimatorContract, StationaryAt100HzDoesNotFall) {
    auto bus=std::make_shared<MessageBus>();
    ErrorStateKalmanFilter ekf(bus, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d(0,0,-9.81), .001);
    ImuStateMessage msg; msg.linear_acceleration=Eigen::Vector3d(0,0,9.81);
    for (int i=0;i<=100;++i) { msg.timestamp_s=i*.01; bus->publish(msg); }
    EXPECT_NEAR(ekf.getEstimatedState().position.norm(), 0, 1e-10);
}
TEST(EstimatorContract, IntegratesOneSecondAt100Hz) {
    auto bus=std::make_shared<MessageBus>();
    ErrorStateKalmanFilter ekf(bus, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d(0,0,-9.81), .001);
    ImuStateMessage msg; msg.linear_acceleration=Eigen::Vector3d(2,0,9.81);
    for (int i=0;i<=100;++i) { msg.timestamp_s=i*.01; bus->publish(msg); }
    EXPECT_NEAR(ekf.getEstimatedState().position.x(), 1, 1e-10);
    EXPECT_NEAR(ekf.getEstimatedState().velocity.x(), 2, 1e-10);
}

#include "rocket_sil_framework/include/plant_models/liquid_engine_model.hpp"
TEST(PropulsionContract, EmptyTankProducesNoThrust) {
    auto bus = std::make_shared<MessageBus>();
    LiquidEngineModel engine(0,5000,4200,2,bus);
    bus->publish(EngineCommandMsg(0,1,true));
    MassProperties mass; mass.propellant_mass=1;
    EXPECT_GT(engine.compute(0,mass,101325).thrust_body.z(),0);
    mass.propellant_mass=0;
    EXPECT_DOUBLE_EQ(engine.compute(1,mass,101325).thrust_body.z(),0);
    EXPECT_DOUBLE_EQ(engine.compute(1,mass,101325).mass_flow_rate,0);
}

#include "rocket_sil_framework/include/core/native_rocket_dynamics_model.hpp"
#include "rocket_sil_framework/include/core/rk4_integrator.hpp"
#include "rocket_sil_framework/include/plant_models/tvc_actuator_model.hpp"
#include "rocket_sil_framework/include/vehicle/rigid_body_mass_model.hpp"
#include "rocket_sil_framework/include/sensors/gps_sensor_model.hpp"
TEST(PropulsionContract, UsesLastPartialFuelInterval) {
    auto bus=std::make_shared<MessageBus>();
    std::vector<std::unique_ptr<IEngineModel>> engines;
    engines.push_back(std::make_unique<LiquidEngineModel>(0,100,100,1,bus));
    std::vector<std::unique_ptr<IActuatorModel>> actuators;
    actuators.push_back(std::make_unique<TvcActuatorModel>(0,0,Eigen::Vector3d::Zero(),bus));
    auto mass=std::make_unique<RigidBodyMassModel>(10,.005,Eigen::Vector3d::Ones(),0,0);
    NativeRocketDynamicsModel model(std::make_unique<RK4Integrator>(),std::move(engines),std::move(actuators),std::move(mass));
    bus->publish(EngineCommandMsg(0,1,true));
    RocketState state; RocketInputs inputs; inputs.force_body.setZero(); inputs.torque_body.setZero(); inputs.gravity_inertial.setZero();
    model.update(.01,inputs,state);
    EXPECT_NEAR(state.velocity.z(),100/10.005*.005,1e-10);
    EXPECT_NEAR(model.getDiagnostics().current_mass_kg,10,1e-10);
    EXPECT_NEAR(model.getDiagnostics().thrust_body.norm(),0,1e-12);
}
TEST(GpsContract, OriginAndDelayAreExplicit) {
    MessageBus bus; GpsSensorModel gps(bus); GpsStateMessage sample; int count=0;
    bus.subscribe<GpsStateMessage>([&](const auto& msg){sample=msg;++count;});
    gps.load_config({{"sensors",{{"gps",{{"update_rate_hz",10},{"delay_ms",50},{"position_noise_std_m",0}}}}},
                     {"location",{{"latitude",28},{"longitude",-80},{"altitude_m",100}}}});
    RocketState state; state.position=Eigen::Vector3d(20,30,40);
    gps.update(.001,0,state); EXPECT_EQ(count,0);
    gps.update(.001,.05,state); ASSERT_EQ(count,1);
    EXPECT_DOUBLE_EQ(sample.timestamp_s,0); EXPECT_DOUBLE_EQ(sample.altitude_m,140);
    EXPECT_NEAR((sample.position_local-state.position).norm(),0,1e-12);
}
