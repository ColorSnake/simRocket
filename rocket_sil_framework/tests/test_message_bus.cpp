#include <gtest/gtest.h>
#include "rocket_sil_framework/include/bus/message_bus.hpp"

struct TestMessageA {
    int value;
};

struct TestMessageB {
    double value;
};

TEST(MessageBusTest, PubSub) {
    MessageBus bus;
    
    int received_a = 0;
    double received_b = 0.0;
    
    bus.subscribe<TestMessageA>([&](const TestMessageA& msg) {
        received_a = msg.value;
    });
    
    bus.subscribe<TestMessageB>([&](const TestMessageB& msg) {
        received_b = msg.value;
    });
    
    TestMessageA msgA{42};
    TestMessageB msgB{3.14};
    
    bus.publish(msgA);
    EXPECT_EQ(received_a, 42);
    EXPECT_EQ(received_b, 0.0);
    
    bus.publish(msgB);
    EXPECT_EQ(received_a, 42);
    EXPECT_EQ(received_b, 3.14);
}
