package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.swerve.TunerConstants.TunerSwerveDrivetrain;

/**
 * CommandSwerveDrivetrain extends the Phoenix 6 SwerveDrivetrain class
 * and implements Subsystem for command-based projects.
 * <p>
 * This class provides:
 * <ul>
 *   <li>SysId routines for translation, steer, and rotation characterization</li>
 *   <li>Automatic operator perspective management (alliance-aware heading)</li>
 *   <li>Simulation support via high-frequency Notifier thread</li>
 *   <li>Vision measurement integration with FPGA timestamp conversion</li>
 * </ul>
 */
@SuppressWarnings("unused")
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.004; // 4 ms (250 Hz)
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Alliance perspective for field-relative driving
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    // SysId characterization requests
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // SysId routine for drive motor characterization
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(4), // reduce step voltage to prevent brownout
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    // SysId routine for steer motor characterization
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(7), // 7V for steer
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    // SysId routine for rotation characterization (heading controller tuning)
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), // rad/s^2 as "volts per second"
            Volts.of(Math.PI),                  // rad/s as "volts"
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    // The currently selected SysId routine
    private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Creates a CommandSwerveDrivetrain with default odometry settings.
     *
     * @param drivetrainConstants drivetrain-wide constants
     * @param modules constants for each swerve module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a CommandSwerveDrivetrain with custom odometry frequency.
     *
     * @param drivetrainConstants drivetrain-wide constants
     * @param odometryUpdateFrequency odometry update frequency (0 = default: 250Hz CAN FD, 100Hz CAN 2.0)
     * @param modules constants for each swerve module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a CommandSwerveDrivetrain with custom standard deviations.
     *
     * @param drivetrainConstants drivetrain-wide constants
     * @param odometryUpdateFrequency odometry update frequency
     * @param odometryStandardDeviation standard deviation for odometry [x, y, theta]
     * @param visionStandardDeviation standard deviation for vision [x, y, theta]
     * @param modules constants for each swerve module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request.
     *
     * @param request function returning the request to apply
     * @return command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * Runs the SysId quasistatic test.
     *
     * @param direction direction of the test
     * @return command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId dynamic test.
     *
     * @param direction direction of the test
     * @return command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Apply operator perspective based on alliance color
        // Only update when disabled to prevent behavior changes mid-match
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at 250Hz for better PID behavior
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement with FPGA timestamp conversion.
     *
     * @param visionRobotPoseMeters the pose from vision
     * @param timestampSeconds the FPGA timestamp of the measurement
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement with custom standard deviations.
     *
     * @param visionRobotPoseMeters the pose from vision
     * @param timestampSeconds the FPGA timestamp of the measurement
     * @param visionMeasurementStdDevs standard deviations [x, y, theta]
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Samples the pose at a given FPGA timestamp.
     *
     * @param timestampSeconds the FPGA timestamp
     * @return the pose at that timestamp, or empty if buffer is empty
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}