#include <gtest/gtest.h>
#include "rocket_sil_framework/include/control/tvc_controller.hpp"
#include <cmath>

TEST(TvcControllerTest, ProportionalControl) {
    auto bus = std::make_shared<MessageBus>();
    TvcController tvc(bus, 1.0, 0.0, 0.5); // Kp=1, Kd=0, max_gimbal=0.5 rad
    
    // Simulate IMU state indicating rocket is pitched by 0.1 rad around Y
    EstimatedStateMessage est;
    std::memset(&est, 0, sizeof(EstimatedStateMessage));
    est.orientation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY());
    est.angular_velocity.setZero();
    
    bus->publish(est);
    tvc.update(0.01);
    
    // Error is roughly -0.1
    // old cmd = Kp * error = 1.0 * (-0.1) = -0.1
    // new cmd with negative feedback = 0.1
    EXPECT_NEAR(tvc.getCmdPitch(), 0.1, 0.01);
    EXPECT_NEAR(tvc.getCmdYaw(), 0.0, 0.01);
}

TEST(TvcControllerTest, Saturation) {
    auto bus = std::make_shared<MessageBus>();
    TvcController tvc(bus, 10.0, 0.0, 0.2); // High Kp, max gimbal = 0.2 rad
    
    EstimatedStateMessage est;
    std::memset(&est, 0, sizeof(EstimatedStateMessage));
    est.orientation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY());
    est.angular_velocity.setZero();
    
    bus->publish(est);
    tvc.update(0.01);
    
    // Command would be 1.0, but should be saturated to 0.2
    EXPECT_NEAR(tvc.getCmdPitch(), 0.2, 0.001);
}
