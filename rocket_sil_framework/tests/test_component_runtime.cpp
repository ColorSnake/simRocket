#include <gtest/gtest.h>
#include "rocket_sil_framework/include/core/phased_scheduler.hpp"
using namespace component;

TEST(ComponentTransport, FutureResultsAndAtomicPayload) {
    struct Axes { int x,y,z; };
    BusTopic<Axes> out("imu.measurement");
    auto in = out.connect("estimator.imu", Policy::latest, unbounded, false);
    out.publish({1,2,3}, 0, 10);
    EXPECT_EQ(in->latest(9).status, Status::missing);
    auto value = in->latest(10);
    EXPECT_EQ(value.status, Status::valid);
    EXPECT_EQ(value.sample->value.z, 3);
    EXPECT_EQ(value.sample->envelope.sample_time, 0);
    EXPECT_EQ(value.sample->envelope.available_time, 10);
}
TEST(ComponentTransport, IndependentCursorsLatestInvalidAndExpiry) {
    BusTopic<int> out("sensor");
    auto latest = out.connect("latest", Policy::latest, 100, false);
    auto all = out.connect("all", Policy::all_samples, 100, false);
    out.publish(1,0,0,10); out.publish(2,1,1,unbounded,Quality::invalid);
    EXPECT_EQ(latest->latest(1).status, Status::invalid);
    auto batch = all->take(1); ASSERT_EQ(batch.size(),2u);
    EXPECT_EQ(batch[0].sample->value,1); EXPECT_EQ(batch[1].status,Status::invalid);
    EXPECT_TRUE(all->take(2).empty());
    out.publish(3,2,2,10);
    EXPECT_EQ(latest->latest(9).status,Status::valid);
    EXPECT_EQ(latest->latest(10).status,Status::stale);
}
TEST(ComponentTransport, LatestMeansDeliveryNotAcquisition) {
    DirectPort<int> out("plant"); auto in=out.connect("input");
    out.publish(1,5,5); out.publish(2,1,10);
    EXPECT_EQ(in->latest(10).sample->value,2);
}
TEST(ComponentTransport, RequiredMissingAgeAndAtomicOverflow) {
    BusTopic<int> out("sensor");
    auto a=out.connect("a",Policy::all_samples,5,true,1);
    auto b=out.connect("b",Policy::all_samples,5,true,2);
    EXPECT_THROW(a->take(0),std::runtime_error);
    out.publish(1,0,0);
    EXPECT_THROW(out.publish(2,1,1),std::runtime_error);
    EXPECT_EQ(b->take(1).size(),1u); // failed fan-out did not reach b
    EXPECT_THROW(a->take(6),std::runtime_error);
    EXPECT_THROW(out.publish(2,3,2),std::invalid_argument);
}
TEST(ComponentTransport, EventsAreExactlyOncePerReceiverAndOnGrid) {
    EventPort<int> events("ignition",10);
    auto a=events.connect("engine_a"), b=events.connect("engine_b");
    events.publish(1,0,10); events.publish(2,0,20);
    EXPECT_TRUE(a->take(9).empty()); EXPECT_EQ(a->take(20).size(),2u);
    EXPECT_TRUE(a->take(30).empty()); EXPECT_EQ(b->take(30).size(),2u);
    EXPECT_THROW(events.publish(3,0,21),std::invalid_argument);
}
TEST(ComponentScheduler, MultirateQueuesAndDeterministicDependencies) {
    BusTopic<int> imu("imu"); auto samples=imu.connect("estimator",Policy::all_samples);
    PhasedScheduler schedule; int produced=0, consumed=0, controlled=0;
    schedule.addTask("controller",100,Phase::compute,{"estimator"},[&](double,double){++controlled;});
    schedule.addTask("estimator",100,Phase::compute,{"imu"},[&](double t,double){consumed+=samples->take(timeFromSeconds(t)).size();});
    schedule.addTask("imu",800,Phase::compute,{},[&](double t,double){imu.publish(++produced,timeFromSeconds(t),timeFromSeconds(t));});
    schedule.compile(.00025);
    for(int tick=0;tick<=400;++tick) schedule.update(tick*.00025,.00025);
    EXPECT_EQ(produced,81); EXPECT_EQ(consumed,81); EXPECT_EQ(controlled,11);
}
TEST(ComponentScheduler, RejectsCyclesAndOrdersEqualTasksById) {
    PhasedScheduler cycle;
    cycle.addTask("a",100,Phase::compute,{"b"},[](double,double){});
    cycle.addTask("b",100,Phase::compute,{"a"},[](double,double){});
    EXPECT_THROW(cycle.compile(.001),std::invalid_argument);
    PhasedScheduler schedule; std::string order;
    schedule.addTask("z",100,Phase::compute,{},[&](double,double){order+='z';});
    schedule.addTask("a",100,Phase::compute,{},[&](double,double){order+='a';});
    schedule.compile(.001);schedule.update(0,.001);EXPECT_EQ(order,"az");
}
TEST(ComponentScheduler, StatefulPlantDoesNotExposeNextStep) {
    DirectPort<double> plant("actuator.state"); auto input=plant.connect("propulsion");
    plant.publish(0,0,0); PhasedScheduler scheduler; double observed=-1;
    scheduler.addTask("observe",100,Phase::observe,{},[&](double t,double){observed=input->latest(timeFromSeconds(t)).sample->value;});
    scheduler.addTask("advance",100,Phase::advance,{},[&](double t,double h){plant.publish(t+h,timeFromSeconds(t+h),timeFromSeconds(t+h));});
    scheduler.compile(.001);
    scheduler.update(0,.001); EXPECT_EQ(observed,0);
    for(int i=1;i<=10;++i)scheduler.update(i*.001,.001);
    EXPECT_DOUBLE_EQ(observed,.01);
}

#include "rocket_sil_framework/include/runtime/simulation_loop.hpp"
TEST(SimulationLoop, ExecutesConfiguredDurationWithoutKnowingModels) {
    RocketState state; state.position.z() = 2;
    int ticks = 0;
    simrocket::runSimulationLoop(state, {.001, 10, .01, 0}, [&](double time, double dt) {
        EXPECT_NEAR(time, ticks * .001, 1e-12);
        state.time = time + dt;
        ++ticks;
    });
    EXPECT_EQ(ticks, 10);
    EXPECT_NEAR(state.time, .01, 1e-12);
}
TEST(SimulationLoop, StopsOnLandingAndClampsGroundState) {
    RocketState state; int ticks = 0;
    simrocket::runSimulationLoop(state, {.001, 100, .1, 0}, [&](double time, double dt) {
        state.time = time + dt;
        state.position.z() = ++ticks == 1 ? 2 : -1;
        state.velocity.z() = -5;
    });
    EXPECT_EQ(ticks, 2);
    EXPECT_DOUBLE_EQ(state.position.z(), 0);
    EXPECT_DOUBLE_EQ(state.velocity.z(), 0);
}
