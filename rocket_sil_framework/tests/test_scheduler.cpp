// Tests for the Scheduler class: verifying execution order, frequency, and
// the dynamic priority assignment from execution_order arrays.

#include <gtest/gtest.h>
#include <vector>
#include <string>
#include "rocket_sil_framework/include/core/scheduler.hpp"

// -----------------------------------------------------------------------
// Test 1: Execution ordering by priority
//   Two tasks with different priorities. Lower number executes first.
//   We track the call sequence in a shared vector.
// -----------------------------------------------------------------------
TEST(SchedulerTest, ExecutionOrderByPriority) {
    Scheduler scheduler;
    std::vector<std::string> call_log;

    // Add in reverse order – scheduler must sort by priority
    scheduler.addTask("task_b", 0.0, 2, [&](double, double) {
        call_log.push_back("B");
    });
    scheduler.addTask("task_a", 0.0, 1, [&](double, double) {
        call_log.push_back("A");
    });
    scheduler.addTask("task_c", 0.0, 3, [&](double, double) {
        call_log.push_back("C");
    });

    scheduler.update(0.0, 0.001);

    ASSERT_EQ(call_log.size(), 3u);
    EXPECT_EQ(call_log[0], "A");
    EXPECT_EQ(call_log[1], "B");
    EXPECT_EQ(call_log[2], "C");
}

// -----------------------------------------------------------------------
// Test 2: Frequency throttling
//   A 10 Hz task running inside a 1000 Hz master loop.
//   After 1 simulated second it should have fired exactly ~10 times.
// -----------------------------------------------------------------------
TEST(SchedulerTest, FrequencyThrottling) {
    Scheduler scheduler;
    int call_count = 0;

    scheduler.addTask("slow_task", 10.0, 1, [&](double, double) {
        ++call_count;
    });

    const double dt = 0.001; // 1000 Hz master
    double sim_time = 0.0;
    for (int i = 0; i < 1000; ++i) {
        scheduler.update(sim_time, dt);
        sim_time += dt;
    }

    // 10 Hz over 1 second → 10 calls (first fires at t=0, then 0.1, 0.2 … 0.9)
    EXPECT_GE(call_count, 10);
    EXPECT_LE(call_count, 11); // Allow small floating-point slip
}

// -----------------------------------------------------------------------
// Test 3: Multi-rate – verify a 100 Hz task fires 10× more than 10 Hz task
// -----------------------------------------------------------------------
TEST(SchedulerTest, MultiRateRatio) {
    Scheduler scheduler;
    int fast_count = 0;
    int slow_count = 0;

    scheduler.addTask("fast", 100.0, 1, [&](double, double) { ++fast_count; });
    scheduler.addTask("slow",  10.0, 2, [&](double, double) { ++slow_count; });

    const double dt = 0.001; // 1000 Hz
    double sim_time = 0.0;
    for (int i = 0; i < 1000; ++i) {
        scheduler.update(sim_time, dt);
        sim_time += dt;
    }

    // Ratio should be ~10:1
    EXPECT_GT(fast_count, 0);
    EXPECT_GT(slow_count, 0);
    EXPECT_NEAR(static_cast<double>(fast_count) / slow_count, 10.0, 1.5);
}

// -----------------------------------------------------------------------
// Test 4: Continuous task (freq = 0) fires every step
// -----------------------------------------------------------------------
TEST(SchedulerTest, ContinuousTaskFiresEveryStep) {
    Scheduler scheduler;
    int count = 0;

    scheduler.addTask("physics", 0.0, 1, [&](double, double) { ++count; });

    const double dt = 0.001;
    double sim_time = 0.0;
    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        scheduler.update(sim_time, dt);
        sim_time += dt;
    }

    EXPECT_EQ(count, steps);
}

// -----------------------------------------------------------------------
// Test 5: Dynamic priority from execution_order array
//   Simulates how main.cpp derives priorities from the JSON array.
// -----------------------------------------------------------------------
TEST(SchedulerTest, DynamicPriorityFromExecutionOrder) {
    std::vector<std::string> exec_order = {"sensors", "control", "dynamics", "telemetry"};

    auto get_priority = [&](const std::string& name) -> uint32_t {
        for (size_t i = 0; i < exec_order.size(); ++i) {
            if (exec_order[i] == name) return static_cast<uint32_t>(i + 1);
        }
        return 999;
    };

    Scheduler scheduler;
    std::vector<std::string> call_log;

    // Register in random order – priorities come from exec_order lookup
    scheduler.addTask("telemetry", 0.0, get_priority("telemetry"), [&](double, double) {
        call_log.push_back("telemetry");
    });
    scheduler.addTask("dynamics",  0.0, get_priority("dynamics"),  [&](double, double) {
        call_log.push_back("dynamics");
    });
    scheduler.addTask("sensors",   0.0, get_priority("sensors"),   [&](double, double) {
        call_log.push_back("sensors");
    });
    scheduler.addTask("control",   0.0, get_priority("control"),   [&](double, double) {
        call_log.push_back("control");
    });
    // An unknown task gets prio 999 → goes last
    scheduler.addTask("custom_fmu", 0.0, get_priority("custom_fmu"), [&](double, double) {
        call_log.push_back("custom_fmu");
    });

    scheduler.update(0.0, 0.001);

    ASSERT_EQ(call_log.size(), 5u);
    EXPECT_EQ(call_log[0], "sensors");
    EXPECT_EQ(call_log[1], "control");
    EXPECT_EQ(call_log[2], "dynamics");
    EXPECT_EQ(call_log[3], "telemetry");
    EXPECT_EQ(call_log[4], "custom_fmu"); // Unknown → last
}
