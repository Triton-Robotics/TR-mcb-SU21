#include "chassis_subsystem.hpp"
#include "tap/architecture/clock.hpp"
#include <cmath>

using tap::arch::clock::getTimeMilliseconds;

namespace tr::control {

    ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers),
        // "front" motor is the motor towards the face the red switch on top faces
        frontMotor(drivers, FRONT_MOTOR_ID, MOTOR_CAN_BUS, false, "front motor"),
        backMotor(drivers, BACK_MOTOR_ID, MOTOR_CAN_BUS, false, "rear motor"),
        frontMotorPID(0.0, 0.0, 0.0), backMotorPID(0.0, 0.0, 0.0), // TODO: Tune these constants
        currentPos(0_m), defaultCommand(drivers, *this), isUsingPid(true), prevTime(getTimeMilliseconds()),
        desiredOutputFront(0), desiredOutputBack(0), desiredShaftRpm(0)
    {
        // Unless another command requiring this subsystem is scheduled,
        Subsystem::setDefaultCommand(&defaultCommand);
    }

    void ChassisSubsystem::initialize() {
        frontMotor.initialize();
        backMotor.initialize();
        // reset prevTime so we don't initially have a huge `dt` for PID controllers, just in case there has been a
        // long delay since the constructor was called
        prevTime = getTimeMilliseconds();
    }

    void ChassisSubsystem::refresh() {
        // Only iterate PID if we're actually using it (i.e., setDesiredOutput has not been called)
        if (isUsingPid) {
            uint32_t dt = prevTime - getTimeMilliseconds();
            auto frontError = (int16_t) (desiredShaftRpm - frontMotor.getShaftRPM());
            auto backError = (int16_t) (desiredShaftRpm - backMotor.getShaftRPM());

            desiredOutputFront = frontMotorPID.iteratePidController(frontError, dt);
            desiredOutputBack = backMotorPID.iteratePidController(backError, dt);
        }
        prevTime = getTimeMilliseconds();
        currentPos = frontMotor.getEncoderUnwrapped() * DISTANCE_PER_ENCODER_TICK;
        frontMotor.setDesiredOutput(desiredOutputFront);
        backMotor.setDesiredOutput(desiredOutputBack);
    }

    void ChassisSubsystem::setDesiredOutput(int16_t desiredOutput) {
        isUsingPid = false;
        desiredOutputFront = desiredOutputBack = desiredOutput;
    }

    void ChassisSubsystem::setDesiredRpm(revolutions_per_minute_t rpm) {
        desiredShaftRpm = units::unit_cast<int32_t>(rpm);
    }
}
