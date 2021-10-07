//
// Created by ariel on 10/2/21.
//

#ifndef TEMPLATE_PROJECT_MOVE_FORWARD_COMMAND_H
#define TEMPLATE_PROJECT_MOVE_FORWARD_COMMAND_H
#include "tap/control/command.hpp"
#include "units.h"

using namespace units::length;
using namespace units::literals;

namespace tr::control {
    class ChassisSubsystem;

    /**
     * MoveDistanceCommand is a command that moves the chassis a set distance from the current position at the time the
     * command is scheduled.
     */
    class MoveDistanceCommand : public tap::control::Command {
    public:
        /**
         * Construct a new MoveDistanceCommand
         * @param chassis A reference to the ChassisSubsystem object
         * @param distance The distance to travel, relative to position at the time this command is scheduled
         */
        MoveDistanceCommand(ChassisSubsystem& chassis, meter_t distance);

        [[nodiscard]] const char *getName() const override;
        /**
         * Resets m_targetPos and m_isFinished
         */
        void initialize() override;
        /**
         * Checks current position and compares it to target position, adjusting velocity if we haven't reached the
         * target position.
         */
        void execute() override;
        /**
         * Ends command execution -- sets m_isFinished to true and sets chassis velocity to 0 if interrupted is true
         * @param interrupted whether or not the command has been interrupted
         */
        void end(bool interrupted) override;
        [[nodiscard]] bool isFinished() const override;

        /**
         * Update the travel distance. Note that if the command is currently scheduled, this does not interrupt it or
         * change the target position, it just saves the travel distance for the next time the command is scheduled.
         * @param distance The distance to travel, relative to starting position when the command is scheduled.
         */
        inline void setTravelDistance(meter_t distance) { m_travelDist = distance; };


    private:
        ChassisSubsystem& chassis;
        bool m_isFinished;
        meter_t m_travelDist;
        meter_t m_targetPos;
    };
}

#endif //TEMPLATE_PROJECT_MOVE_FORWARD_COMMAND_H
