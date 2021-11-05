//
// Created by ariel on 10/8/21.
//

#ifndef SENTRY_TURRET_SUBSYSTEM_HPP
#define SENTRY_TURRET_SUBSYSTEM_HPP

#include "algo/simple_pid_controller.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "units.h"

using namespace units::literals;
using units::angle::radian_t;
using tap::motor::DjiMotor;
using tr::algo::SimplePIDController;

namespace tr::control::turret {
    class TurretSubsystem : public tap::control::Subsystem {
    public:
        explicit TurretSubsystem(tap::Drivers *drivers);

        TurretSubsystem(const TurretSubsystem &other) = delete;

        TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

        ~TurretSubsystem() override = default;

        void initialize() override;

        void refresh() override;

        mockable inline void setTargetPosition(radian_t rotation, radian_t inclination);
        mockable inline void setTargetRotation(radian_t rotation);
        mockable inline void setTargetInclination(radian_t inclination);
        mockable inline void fire();

        [[nodiscard]] mockable inline std::tuple<radian_t, radian_t> getCurrentPosition();
        [[nodiscard]] mockable inline radian_t getCurrentRotation();
        [[nodiscard]] mockable inline radian_t getCurrentInclination();

    private:
        static constexpr tap::motor::MotorId ROTATION_MOTOR_ID = tap::motor::MOTOR1;
        static constexpr tap::motor::MotorId INCLINATION_MOTOR_ID = tap::motor::MOTOR2;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;
        static constexpr radian_t RADIANS_PER_ENCODER_TICK = 2_rad*units::constants::pi / DjiMotor::ENC_RESOLUTION;

        DjiMotor rotationMotor;
        DjiMotor inclinationMotor;
        SimplePIDController<int32_t> rotationPID;
        SimplePIDController<int32_t> inclinationPID;

        bool shouldFire;
        radian_t targetRotation;
        radian_t targetInclination;
        uint32_t prevTime;
    };
}


#endif //SENTRY_TURRET_SUBSYSTEM_HPP
