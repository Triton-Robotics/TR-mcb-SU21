//
// Created by ariel on 10/8/21.
//

#include "turret_subsystem.hpp"

using tap::arch::clock::getTimeMilliseconds;

const auto M_2PI_RAD = units::angle::radian_t(M_2_PI);
namespace tr::control::turret {
    TurretSubsystem::TurretSubsystem(tap::Drivers *drivers) : tap::control::Subsystem(drivers),
                                                              rotationMotor(drivers, ROTATION_MOTOR_ID, MOTOR_CAN_BUS, false, "rotation motor"),
                                                              inclinationMotor(drivers, INCLINATION_MOTOR_ID, MOTOR_CAN_BUS, false, "inclination motor"),
                                                              rotationPID(0.0, 0.0, 0.0), inclinationPID(0.0, 0.0, 0.0),
                                                              shouldFire(false), targetRotation(0.0_rad), targetInclination(0.0_rad), prevTime(0)
    {

    }

    void TurretSubsystem::initialize() {
        rotationMotor.initialize();
        inclinationMotor.initialize();
        prevTime = tap::arch::clock::getTimeMilliseconds();
    }

    void TurretSubsystem::refresh() {
        uint32_t currentTime = getTimeMilliseconds();
        uint32_t dt = prevTime - currentTime;
        prevTime = currentTime;

        auto rotationError = units::unit_cast<int16_t>(targetRotation - getCurrentRotation());
        auto inclinationError = units::unit_cast<int16_t>(targetInclination - getCurrentInclination());

        rotationMotor.setDesiredOutput(rotationPID.iterate(rotationError, dt));
        inclinationMotor.setDesiredOutput(inclinationPID.iterate(inclinationError, dt));

        if (shouldFire) {
            // TODO: Write code to fire turret
        }
    }

    void TurretSubsystem::setTargetPosition(radian_t rotation, radian_t inclination) {
        setTargetRotation(rotation);
        setTargetInclination(inclination);
    }

    void TurretSubsystem::setTargetRotation(radian_t rotation) {
        targetRotation = rotation;
        normalizeRotation();
    }

    void TurretSubsystem::setTargetInclination(radian_t inclination) {
        targetInclination = std::clamp(inclination, INCLINATION_MIN, INCLINATION_MAX);
    }

    void TurretSubsystem::fire() {
        shouldFire = true;
    }

    std::tuple<radian_t, radian_t> TurretSubsystem::getCurrentPosition() {
        return std::make_pair<radian_t, radian_t>(getCurrentRotation(), getCurrentInclination());
    }

    radian_t TurretSubsystem::getCurrentRotation() {
        return rotationMotor.getEncoderUnwrapped() * RADIANS_PER_ENCODER_TICK;
    }

    radian_t TurretSubsystem::getCurrentInclination() {
        return inclinationMotor.getEncoderUnwrapped() * RADIANS_PER_ENCODER_TICK;
    }

    /// This is an expensive operation, try not to do it too much
    void TurretSubsystem::normalizeRotation() {
        if (units::math::fabs(targetRotation) > M_2PI_RAD) {
            targetRotation = units::math::fmod(targetRotation, M_2PI_RAD);
        }
        // Look ma, no branches!
        targetRotation = targetRotation + (M_2PI_RAD*(targetRotation > 0_rad));
    }
}