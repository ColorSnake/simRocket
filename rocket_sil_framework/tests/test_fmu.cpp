#include <gtest/gtest.h>
#include "rocket_sil_framework/include/core/fmu_component.hpp"
#include "rocket_sil_framework/include/core/scheduler.hpp"

TEST(FmiRuntime, StatefulModelAccumulatesCorrectTimeAndParameters) {
    FmuModel model(TEST_INTEGRATOR_FMU, "integration");
    model.initialize({{"gain",3}}, {{"drive",2}}, 1);
    for(int i=0;i<100;++i) model.step(i*.01,.01);
    EXPECT_NEAR(model.getReal("integral"),6,1e-12);
    EXPECT_THROW(model.step(.9,.01),std::invalid_argument);
    EXPECT_THROW(model.setInputs({{"gain",5}}),std::invalid_argument);
}
TEST(FmiRuntime, SameNameInstancesHaveIndependentResources) {
    auto a=std::make_unique<FmuModel>(TEST_INTEGRATOR_FMU,"same");
    FmuModel b(TEST_INTEGRATOR_FMU,"same");
    a->initialize({},{{"drive",1}},1); b.initialize({},{{"drive",2}},1);
    a.reset(); b.step(0,.1); EXPECT_NEAR(b.getReal("integral"),.2,1e-12);
}
TEST(FmiRuntime, RejectsBadMappingAndFailedStep) {
    FmuModel model(TEST_INTEGRATOR_FMU,"errors");
    EXPECT_THROW(model.variable("missing"),std::invalid_argument);
    model.initialize({{"gain",-1}},{{"drive",1}},1);
    EXPECT_THROW(model.step(0,.01),std::runtime_error);
    EXPECT_THROW(model.step(0,.01),std::invalid_argument);
}
TEST(FmiComponent, ArbitraryNamesAndNoFutureOutput) {
    SignalRegistry signals; signals.declare("source", "m/s", 2);
    nlohmann::json config={{"path",TEST_INTEGRATOR_FMU}, {"inputs",{{"drive","source"}}},
        {"outputs",{{"integral","result"}}}, {"parameters",{{"gain",3}}}};
    FmuComponent component("custom",config,signals); component.initialize(1);
    Scheduler scheduler;
    scheduler.addTask("custom",100,0,[&](double t,double dt){component.step(t,dt);});
    for(int i=0;i<1000;++i) {
        double t=i*.001; component.publishReady(t); scheduler.update(t,.001);
        EXPECT_LE(signals.get("result").time,t+1e-12);
        EXPECT_NEAR(signals.get("result").value,6*signals.get("result").time,1e-10);
    }
    EXPECT_TRUE(component.publishReady(1)); EXPECT_NEAR(signals.get("result").value,6,1e-10);
}
TEST(FmiRuntime, NativeImuEquivalentWithoutNoise) {
    FmuModel model(TEST_IMU_FMU,"imu");
    model.initialize({{"param.gyro_noise_std",0},{"param.accel_noise_std",0}},
        {{"in.gyro_x",.3},{"in.accel_z",9.81}},1);
    model.step(0,.01);
    EXPECT_DOUBLE_EQ(model.getReal("out.gyro_x"),.3);
    EXPECT_DOUBLE_EQ(model.getReal("out.accel_z"),9.81);
}

TEST(FmiComponent, LatchedMeasurementKeepsAcquisitionTime) {
    SignalRegistry signals; signals.declare("source", "rad/s", 2);
    nlohmann::json config={{"path",TEST_INTEGRATOR_FMU}, {"inputs",{{"drive","source"}}},
        {"outputs",{{"integral","result"}}}, {"sample_time_policy","latched_input"}, {"discard_repeated_samples",true}};
    FmuComponent component("time",config,signals); component.initialize(1);
    ASSERT_TRUE(component.publishReady(0));
    component.step(0,.01);
    EXPECT_FALSE(component.publishReady(.005));
    EXPECT_FALSE(component.publishReady(.01)); // initial measurement at t=0 already delivered
    signals.set("source",2,.01);component.step(.01,.01);
    ASSERT_TRUE(component.publishReady(.02));
    EXPECT_DOUBLE_EQ(component.sampleTime(),.01);
    EXPECT_DOUBLE_EQ(component.availableTime(),.02);
    EXPECT_DOUBLE_EQ(signals.get("result").time,.01);
}
