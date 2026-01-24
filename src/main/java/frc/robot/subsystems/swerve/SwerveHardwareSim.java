package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;

/**
 * Implements {@link SwerveHardware} for simulation.
 * <p>
 * Simulates swerve module behavior by integrating commanded states
 * over time. No physics simulation is performed - modules instantly
 * achieve their desired states.
 */
public class SwerveHardwareSim implements SwerveHardware {

    private Rotation2d heading = new Rotation2d();

    public SwerveHardwareSim() {
        // command all wheels to face forward on startup
        lockTurnMotors();
    }
    private double yawRateDps = 0.0;

    private final SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    @Override
    public Rotation2d getHeading() {
        return heading;
    }

    @Override
    public double getYawRateDps() {
        return yawRateDps;
    }

    @Override
    public void zeroHeading() {
        heading = new Rotation2d();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return moduleStates;
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = states[i];

            // integrate velocity to update position
            double distanceDelta = states[i].speedMetersPerSecond * Util.DT;
            modulePositions[i] = new SwerveModulePosition(
                    modulePositions[i].distanceMeters + distanceDelta,
                    states[i].angle);
        }

        // update heading based on chassis rotation
        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(states);
        yawRateDps = Math.toDegrees(speeds.omegaRadiansPerSecond);
        heading = heading.plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Util.DT));
    }

    @Override
    public void setX() {
        moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    }

    @Override
    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(0, modulePositions[i].angle);
        }
    }

    //==========================================================================
    // SysId characterization support
    //==========================================================================

    /** Simple voltage-to-velocity conversion factor (m/s per volt) */
    private static final double KV_SIM = 0.4;  // ~4.8 m/s at 12V

    @Override
    public void setDriveVoltage(double volts) {
        // simulate velocity proportional to voltage
        double velocity = volts * KV_SIM;

        // create states with the simulated velocity, all wheels forward
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(velocity, moduleStates[i].angle);
        }
        setModuleStates(states);
    }

    @Override
    public void lockTurnMotors() {
        // point all wheels forward (0 degrees)
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = new SwerveModuleState(moduleStates[i].speedMetersPerSecond, new Rotation2d());
            modulePositions[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, new Rotation2d());
        }
    }
}
