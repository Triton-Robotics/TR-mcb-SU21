#ifndef SENTRY_CHASSIS_SUBSYSTEM_COMMAND_H
#define SENTRY_CHASSIS_SUBSYSTEM_COMMAND_H

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "algo/simple_pid_controller.hpp"
#include "units.h"
#include "wiggle_command.hpp"

using namespace units::literals;
using namespace units::length;
using namespace units::angular_velocity;
using units::velocity::meters_per_second_t;
using units::angle::radian_t;
using tap::motor::DjiMotor;

namespace tr::control {
class ChassisSubsystem : public tap::control::Subsystem {
    public:
        /**
         * Construct the chassis subsystem. Does not initialize motors.
         * @param drivers Pointer to the global drivers object
         */
        explicit ChassisSubsystem(tap::Drivers* drivers);

        ChassisSubsystem(const ChassisSubsystem& other) = delete;
        ChassisSubsystem &operator=(const ChassisSubsystem& other) = delete;
        ~ChassisSubsystem() override = default;

        /**
         * Initialize the chassis subsystem, including initializing motors.
         */
        void initialize() override;
        void refresh() override;

        /**
         * Set the desired shaft RPM for both motors. This will change the motor outputs by way of each motor's
         * respective PID controller.
         * @param rpm The desired shaft RPM. Note that any angular velocity unit type can be passed here, and will be
         *  implicitly converted to rev/min
         */
        mockable void setDesiredRpm(revolutions_per_minute_t rpm);

        /**
         * Set the desired chassis velocity. This will calculate the requisite shaft RPM using WHEEL_RADIUS_MM and then
         * call `ChassisSubsystem::setDesiredRpm(calculated_rpm)`
         * @param velocity The desired chassis velocity.
         */
        mockable inline void setDesiredVelocity(meters_per_second_t v) {
            // The math below converts linear speed (m/s) to angular speed in rad/s, which is implicitly converted to
            // rpm when passed to setDesiredRpm by the `units` library
            auto angular_vel = v / WHEEL_RADIUS * 1_rad;
            static_assert(units::traits::is_angular_velocity_unit<decltype(angular_vel)>::value,
                    "Computed angular velocity does not have unit representing angular velocity.");
            setDesiredRpm(angular_vel);
        };

        /**
         * Sets the desired motor output values directly, bypassing the PID controllers. This will override any desired
         * RPM / velocity set in setDesiredRpm or setDesiredVelocity, and will disable the PID controllers until either
         * of the aforementioned methods is called again.
         * @param desiredOutput The desired motor output.
         */
        mockable void setDesiredOutput(int32_t desiredOutput);

        /**
         * Returns the current position of the robot, calculated by multiplying the net encoder position since motor
         * initialization by `DISTANCE_PER_ENCODER_TICK`
         * @return The current linear position of the robot
         */
        [[nodiscard]] mockable inline meter_t getCurrentPos() const { return currentPos; };

        // getters
        const DjiMotor &getFrontMotor() const;
        const DjiMotor &getBackMotor() const;
        bool getPidState() const;
        int32_t getDesiredOutputFront() const;
        int32_t getDesiredOutputBack() const;
        revolutions_per_minute_t getDesiredShaftRpm() const;

    private:
        static constexpr tap::motor::MotorId FRONT_MOTOR_ID = tap::motor::MOTOR2;
        static constexpr tap::motor::MotorId BACK_MOTOR_ID = tap::motor::MOTOR3;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
        static constexpr meter_t WHEEL_RADIUS = 60_mm; //TODO: make this actual value
        static constexpr meter_t DISTANCE_PER_ENCODER_TICK = 2.0*units::constants::pi/ DjiMotor::ENC_RESOLUTION * WHEEL_RADIUS;
        static constexpr int32_t MOTOR_MAX = 3000;
        // motor is rated for 469 rpm (nice), but we'll limit it slightly lower just to be safe
        static constexpr revolutions_per_minute_t RPM_MAX = 425_rpm;

        DjiMotor frontMotor;
        DjiMotor backMotor;

        tr::algo::SimplePIDController<int32_t> frontMotorPID;
        tr::algo::SimplePIDController<int32_t> backMotorPID;

        meter_t currentPos;

        WiggleCommand defaultCommand;

        bool isUsingPid;
        uint32_t prevTime;
        int32_t desiredOutputFront;
        int32_t desiredOutputBack;
        int32_t desiredShaftRpm;
    };
}

#endif //SENTRY_CHASSIS_SUBSYSTEM_COMMAND_H