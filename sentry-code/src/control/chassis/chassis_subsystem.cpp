#include "chassis_subsystem.hpp"
#include "tap/architecture/clock.hpp"
#include <cmath>

using tap::arch::clock::getTimeMilliseconds;

namespace tr::control {
    ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers),
        // "front" motor is the motor towards the face the red switch on top faces
        frontMotor(drivers, FRONT_MOTOR_ID, MOTOR_CAN_BUS, false, "front motor"),
        backMotor(drivers, BACK_MOTOR_ID, MOTOR_CAN_BUS, false, "rear motor"),
        frontMotorPID(5.0, 0.0, 0.0), backMotorPID(5.0, 0.0, 0.0),
        currentPos(0_m), defaultCommand(drivers, *this), isUsingPid(true), prevTime(getTimeMilliseconds()),
        desiredOutputFront(0), desiredOutputBack(0), desiredShaftRpm(0)
    {
        // Unless another command requiring this subsystem is scheduled,
        Subsystem::setDefaultCommand(&defaultCommand);
    }

    void ChassisSubsystem::initialize() {
        frontMotor.initialize();
        backMotor.initialize();
        defaultCommand.initialize();
        // reset prevTime so we don't initially have a huge `dt` for PID controllers, just in case there has been a
        // long delay since the constructor was called
        prevTime = getTimeMilliseconds();
    }

    void ChassisSubsystem::refresh() {
        // Only iterate PID if we're actually using it (i.e., setDesiredOutput has not been called)
        uint32_t currentTime = getTimeMilliseconds();
        if (isUsingPid) {
            uint32_t dt = prevTime - currentTime;
            auto frontError = (int16_t) (desiredShaftRpm - frontMotor.getShaftRPM());
            auto backError = (int16_t) (desiredShaftRpm - backMotor.getShaftRPM());

            desiredOutputFront = frontMotorPID.iterate(frontError, dt);
            desiredOutputBack = backMotorPID.iterate(backError, dt);

            // Limit output values
            desiredOutputBack = std::clamp(desiredOutputBack, -MOTOR_MAX, MOTOR_MAX);
            desiredOutputFront = std::clamp(desiredOutputFront, -MOTOR_MAX, MOTOR_MAX);
        }
        prevTime = currentTime;
        currentPos = frontMotor.getEncoderUnwrapped() * DISTANCE_PER_ENCODER_TICK;


        // set motor outputs
        frontMotor.setDesiredOutput(desiredOutputFront);
        backMotor.setDesiredOutput(desiredOutputBack);
    }

    void ChassisSubsystem::setDesiredOutput(int32_t desiredOutput) {
        isUsingPid = false;
        desiredOutputFront = desiredOutputBack = std::clamp(desiredOutput, -MOTOR_MAX, MOTOR_MAX);
    }

    void ChassisSubsystem::setDesiredRpm(revolutions_per_minute_t rpm) {
        rpm = std::clamp(rpm, -RPM_MAX, RPM_MAX);
        isUsingPid = true;
        prevTime = getTimeMilliseconds();
        // we use int32_t to deal with rpm/motor stuff internally, so convert here
        desiredShaftRpm = units::unit_cast<int32_t>(rpm);
    }

    const DjiMotor &ChassisSubsystem::getFrontMotor() const {
        return frontMotor;
    }

    const DjiMotor &ChassisSubsystem::getBackMotor() const {
        return backMotor;
    }

    bool ChassisSubsystem::getPidState() const {
        return isUsingPid;
    }

    int32_t ChassisSubsystem::getDesiredOutputFront() const {
        return desiredOutputFront;
    }

    int32_t ChassisSubsystem::getDesiredOutputBack() const {
        return desiredOutputBack;
    }

    revolutions_per_minute_t ChassisSubsystem::getDesiredShaftRpm() const {
        return revolutions_per_minute_t(desiredShaftRpm);
    }
}
