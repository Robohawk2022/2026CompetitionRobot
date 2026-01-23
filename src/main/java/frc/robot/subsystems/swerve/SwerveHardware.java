package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Hardware interface for a swerve drive system.
 * <p>
 * Implementations include {@link SwerveHardwareCTRE} for real hardware
 * and {@link SwerveHardwareSim} for simulation.
 */
public interface SwerveHardware {

    /**
     * @return the current heading of the robot from the gyro
     */
    Rotation2d getHeading();

    /**
     * @return the angular velocity (yaw rate) in degrees per second
     */
    double getYawRateDps();

    /**
     * Zeroes the gyro heading.
     */
    void zeroHeading();

    /**
     * Sets the gyro to a specific heading.
     * @param heading the heading to set
     */
    void setHeading(Rotation2d heading);

    /**
     * @return an array of the current module positions (distance traveled + angle)
     */
    SwerveModulePosition[] getModulePositions();

    /**
     * @return an array of the current module states (velocity + angle)
     */
    SwerveModuleState[] getModuleStates();

    /**
     * Sets the desired states for all swerve modules.
     * @param states the desired states (FL, FR, BL, BR)
     */
    void setModuleStates(SwerveModuleState[] states);

    /**
     * Sets the wheels to an X pattern to prevent the robot from moving.
     */
    void setX();

    /**
     * Resets the drive encoders to zero.
     */
    void resetEncoders();

    /**
     * Refreshes all hardware signals in a single CAN transaction.
     * Call once per periodic cycle before reading encoder/gyro values.
     * Default implementation does nothing (for simulation).
     */
    default void refreshSignals() {
        // no-op for simulation
    }
}
