//
// Created by ariel on 10/2/21.
//

#include "move_distance_command.hpp"
#include "chassis_subsystem.hpp"

using namespace units::literals;
using tap::control::Subsystem;
namespace tr::control {
    MoveDistanceCommand::MoveDistanceCommand(ChassisSubsystem &chassis, meter_t distance)
            : chassis(chassis), m_isFinished(false), m_travelDist(distance), m_targetPos(0) {
        Command::addSubsystemRequirement((Subsystem*) &chassis);
    }

    const char *MoveDistanceCommand::getName() const {
        return "MoveDistanceCommand";
    }

    void MoveDistanceCommand::initialize() {
        m_targetPos = chassis.getCurrentPos() + m_travelDist;
        m_isFinished = false;
    }

    void MoveDistanceCommand::execute() {
        // TODO: Write a real implementation here
        auto diff = m_targetPos - chassis.getCurrentPos();
        if (diff > 0.1_m) {
            chassis.setDesiredVelocity(-1_mph);
        } else if (diff < -0.1_m) {
            chassis.setDesiredVelocity(1_mph);
        } else {
            chassis.setDesiredVelocity(0_mph);
            m_isFinished = true;
        }
    }

    void MoveDistanceCommand::end(bool interrupted) {
        if (interrupted) {
            chassis.setDesiredVelocity(0_mph);
        }
        m_isFinished = true;
    }

    bool MoveDistanceCommand::isFinished() const {
        return m_isFinished;
    }
}
