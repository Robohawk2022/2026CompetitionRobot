package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Enumeration representing the walls of the arena
 */
public enum ArenaWall {

    NORTH,
    SOUTH,
    EAST,
    WEST;

    /**
     * @return the heading the robot will have when it's facing the
     * specified wall
     */
    public Rotation2d getFacingHeading() {
        return switch (this) {
            case EAST -> Rotation2d.kZero;
            case WEST -> Rotation2d.k180deg;
            case NORTH -> Rotation2d.kCCW_90deg;
            case SOUTH -> Rotation2d.kCW_90deg;
        };
    }
}
