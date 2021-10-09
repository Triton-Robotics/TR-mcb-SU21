//
// Created by ariel on 10/6/21.
//

#include "wiggle_command.hpp"
#include "chassis_subsystem.hpp"

namespace tr::control {
    WiggleCommand::WiggleCommand(tap::Drivers *drivers, ChassisSubsystem &chassis) : tap::control::ComprisedCommand(drivers),
    m_isFinished(false), moveForwardCommand(chassis, 1_ft), moveBackwardCommand(chassis, -1_ft),
    currentCommand(&moveForwardCommand), nextCommand(&moveBackwardCommand), chassis(chassis)
    {
        Command::addSubsystemRequirement(&chassis);
    }

    void WiggleCommand::initialize() {
        m_isFinished = false;
        currentCommand = &moveForwardCommand;
        nextCommand = &moveBackwardCommand;

        comprisedCommandScheduler.addCommand(currentCommand);
    }

    void WiggleCommand::execute() {
        // if current command is finished, switch the command pointers and schedule the next one
        if (currentCommand->isFinished()) {
            auto temp = nextCommand;
            nextCommand = currentCommand;
            currentCommand = temp;
            comprisedCommandScheduler.addCommand(currentCommand);
        }
        comprisedCommandScheduler.run();
    }

    void WiggleCommand::end(bool interrupted) {
        comprisedCommandScheduler.removeCommand(currentCommand, interrupted);
    }
}