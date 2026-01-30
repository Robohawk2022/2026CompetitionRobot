package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootingCommands {

    /**
     * Represents a type of shot, which corresponds to a specific distance
     * from the center of the hub
     */
    public enum ShotType {

        POINT_BLANK,
        CLOSE_RANGE,
        LONG_RANGE;

        /**
         * @return the distance in feet from the hub center for this type of shot
         */
        public double distance() {
            throw new UnsupportedOperationException();
        }
    }

    /**
     * @param robot the robot
     * @param shotType the type of shot
     * @return a command that will run the shooter until interrupted, to
     * unload all fuel in the hopper
     */
    public static Command vomitCommand(RobotContainer robot, ShotType shotType) {
        throw new UnsupportedOperationException();
    }

    /**
     * @param robot the robot
     * @param shotType the type of shot
     * @return a command that will drive to the shooting position and unload
     * all fuel in the hopper
     */
    public static Command makeShotCommand(RobotContainer robot, ShotType shotType) {
        throw new UnsupportedOperationException();
    }
}
