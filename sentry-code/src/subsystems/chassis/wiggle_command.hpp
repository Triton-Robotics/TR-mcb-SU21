//
// Created by ariel on 10/6/21.
//

#ifndef SENTRY_WIGGLE_COMMAND_H
#define SENTRY_WIGGLE_COMMAND_H

#include "tap/control/comprised_command.hpp"
#include "move_distance_command.hpp"

namespace tr::control {
    class ChassisSubsystem;
    /**
     * A command that moves the chassis back and forth a set distance (currently 1 ft) at a set speed (currently 1 mph)
     */
    class WiggleCommand : public tap::control::ComprisedCommand {
    public:
        /**
         * Initializes all variables and calls Command::addSubsystemRequirement(&chassis)
         * @param drivers The global drivers pointer
         * @param chassis A reference to the chassis instance
         */
        explicit WiggleCommand(tap::Drivers *drivers, ChassisSubsystem& chassis);

        [[nodiscard]] const char *getName() const override { return "WiggleCommand"; };
        /**
         * Sets m_isFinished to false, sets up command-switching pointers, and schedules the first command with
         * comprisedCommandScheduler
         */
        void initialize() override;
        /**
         * Checks if current move command is finished, and switches the scheduled command if it is
         */
        void execute() override;
        void end(bool interrupted) override;
        [[nodiscard]] inline bool isFinished() const override { return m_isFinished; };

    private:
        bool m_isFinished;

        MoveDistanceCommand moveForwardCommand;
        MoveDistanceCommand moveBackwardCommand;
        MoveDistanceCommand* currentCommand;
        MoveDistanceCommand* nextCommand;

        ChassisSubsystem& chassis;
    };
}



#endif //SENTRY_WIGGLE_COMMAND_H
