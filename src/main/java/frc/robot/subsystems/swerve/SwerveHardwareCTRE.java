package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

import static frc.robot.Config.Swerve.*;

/**
 * Implements {@link SwerveHardware} using CTRE TalonFX motors,
 * CANcoders, and a Pigeon2 gyro.
 */
public class SwerveHardwareCTRE implements SwerveHardware {

    /** Update frequency for high-frequency odometry signals (Hz) */
    private static final double HF_UPDATE_FREQUENCY = 250.0;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yawSignal;
    private final BaseStatusSignal[] allSignals;

    // cached heading (updated in refreshSignals)
    private Rotation2d cachedHeading = new Rotation2d();

    /**
     * Creates the CTRE swerve hardware using CAN IDs from Config.
     */
    public SwerveHardwareCTRE() {
        frontLeft = new SwerveModule(
                "FL",
                FL_DRIVE_ID,
                FL_TURN_ID,
                FL_ENCODER_ID,
                FL_ANGULAR_OFFSET,
                FL_DRIVE_INVERTED);

        frontRight = new SwerveModule(
                "FR",
                FR_DRIVE_ID,
                FR_TURN_ID,
                FR_ENCODER_ID,
                FR_ANGULAR_OFFSET,
                FR_DRIVE_INVERTED);

        backLeft = new SwerveModule(
                "BL",
                BL_DRIVE_ID,
                BL_TURN_ID,
                BL_ENCODER_ID,
                BL_ANGULAR_OFFSET,
                BL_DRIVE_INVERTED);

        backRight = new SwerveModule(
                "BR",
                BR_DRIVE_ID,
                BR_TURN_ID,
                BR_ENCODER_ID,
                BR_ANGULAR_OFFSET,
                BR_DRIVE_INVERTED);

        pigeon = new Pigeon2(PIGEON_ID);
        configurePigeon();

        // configure yaw signal for high-frequency reads
        yawSignal = pigeon.getYaw();
        yawSignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);

        // collect all signals for synchronized reads (13 signals total):
        // 12 module signals (drive position, drive velocity, turn per module) + 1 gyro yaw
        BaseStatusSignal[] flSignals = frontLeft.getSignals();
        BaseStatusSignal[] frSignals = frontRight.getSignals();
        BaseStatusSignal[] blSignals = backLeft.getSignals();
        BaseStatusSignal[] brSignals = backRight.getSignals();

        allSignals = new BaseStatusSignal[flSignals.length + frSignals.length + blSignals.length + brSignals.length + 1];
        int idx = 0;
        for (BaseStatusSignal signal : flSignals) allSignals[idx++] = signal;
        for (BaseStatusSignal signal : frSignals) allSignals[idx++] = signal;
        for (BaseStatusSignal signal : blSignals) allSignals[idx++] = signal;
        for (BaseStatusSignal signal : brSignals) allSignals[idx++] = signal;
        allSignals[idx] = yawSignal;

        // initial refresh to populate cached values
        refreshSignals();
    }

    private void configurePigeon() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);
    }

    @Override
    public Rotation2d getHeading() {
        return cachedHeading;
    }

    @Override
    public void refreshSignals() {
        // batch refresh all signals in a single CAN transaction
        BaseStatusSignal.refreshAll(allSignals);

        // update cached values in each module
        frontLeft.refreshSignals();
        frontRight.refreshSignals();
        backLeft.refreshSignals();
        backRight.refreshSignals();

        // update cached heading from the yaw signal (yaw is in degrees)
        cachedHeading = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    }

    @Override
    public double getYawRateDps() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    @Override
    public void zeroHeading() {
        pigeon.setYaw(0);
    }

    @Override
    public void setHeading(Rotation2d heading) {
        pigeon.setYaw(heading.getDegrees());
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    @Override
    public void resetEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }
}
