//
// Created by ariel on 11/15/21.
//
#include <gtest/gtest.h>
#include <random>

// pretty sure this gets set from command line when we run `scons run-tests`,
// but if we don't set it here then clion complains about the default constructor for
// `tap::Drivers` not existing
#define ENV_UNIT_TESTS

#include "tap/drivers.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "units.h"

using namespace tr::control;
using namespace units::literals;

class ChassisSubsystemTest : public ::testing::Test {
protected:
    ChassisSubsystemTest() : cs(&drivers) {}

    tap::Drivers drivers;
    ChassisSubsystem cs;
};

TEST_F(ChassisSubsystemTest, setDesiredOutputDisablesPid) {
    cs.setDesiredOutput(100);
    ASSERT_FALSE(cs.getPidState());
}

TEST_F(ChassisSubsystemTest, setDesiredRpmEnablesPid) {
    cs.setDesiredOutput(100);
    ASSERT_FALSE(cs.getPidState());
    cs.setDesiredRpm(100_rpm);
    ASSERT_TRUE(cs.getPidState());
}

const int32_t CHASSIS_MOTOR_MAX = 3000;
const int32_t ARBITRARY_LARGE_VALUE = 10000000;
TEST_F(ChassisSubsystemTest, setDesiredOutputClampsPositive) {
    cs.setDesiredOutput(ARBITRARY_LARGE_VALUE);
    EXPECT_EQ(cs.getDesiredOutputBack(), CHASSIS_MOTOR_MAX);
    EXPECT_EQ(cs.getDesiredOutputFront(), CHASSIS_MOTOR_MAX);
}

TEST_F(ChassisSubsystemTest, setDesiredOutputClampsNegative) {
    cs.setDesiredOutput(-ARBITRARY_LARGE_VALUE);
    EXPECT_EQ(cs.getDesiredOutputFront(), -CHASSIS_MOTOR_MAX);
    EXPECT_EQ(cs.getDesiredOutputBack(), -CHASSIS_MOTOR_MAX);
}

const auto CHASSIS_MAX_RPM = 425_rpm;
const auto ARBITRARY_LARGE_RPM = 10000000_rpm;
TEST_F(ChassisSubsystemTest, setDesiredRpmClampsPositive) {
    cs.setDesiredRpm(ARBITRARY_LARGE_RPM);
    EXPECT_EQ(cs.getDesiredShaftRpm(), CHASSIS_MAX_RPM);
}

TEST_F(ChassisSubsystemTest, setDesiredRpmClampsNegative) {
    cs.setDesiredRpm(-ARBITRARY_LARGE_RPM);
    EXPECT_EQ(cs.getDesiredShaftRpm(), -CHASSIS_MAX_RPM);
}

const auto WHEEL_RADIUS = 60_mm;
TEST_F(ChassisSubsystemTest, setDesiredVelocitySetsCorrectRPM) {
    // seed random number generator
    std::srand(std::time(nullptr));
    // generate a bunch of RPM values, set them, and then make sure the result is as expected
    for (int i = 0; i < 1000; i++) {
        // generates random numbers in the range [-1e6, 1e6]
        auto linear_vel = meters_per_second_t(std::rand() % 2000000 - 1000000);
        cs.setDesiredVelocity(linear_vel);
        // linear_vel/WHEEL_RADIUS produces a value with type 1/1_sec; although by SI definition this is equivalent to 1_rad/s,
        // the units library doesn't recognize that; thus, we multiply by 1_rad to make the units into an angular vel type
        auto angular_vel = revolutions_per_minute_t(linear_vel / WHEEL_RADIUS * 1_rad);
        auto expected_angular = std::clamp(angular_vel, -CHASSIS_MAX_RPM, CHASSIS_MAX_RPM);
        EXPECT_EQ(expected_angular, cs.getDesiredShaftRpm());
    }
}
