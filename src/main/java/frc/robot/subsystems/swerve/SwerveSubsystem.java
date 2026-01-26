package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightEstimator;
import frc.robot.subsystems.vision.VisionEstimateResult;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;
import static frc.robot.Config.Odometry.enableDiagnostics;
import static frc.robot.Config.Odometry.driftWarning;
import static frc.robot.Config.Orbit.*;

/**
 * Swerve drive subsystem with odometry and pose estimation.
 * <p>
 * Adapted from Robohawk2022/2025CompetitionRobot, converted from
 * REV SparkMax to CTRE TalonFX.
 */
public class SwerveSubsystem extends SubsystemBase {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    private final SwerveHardware hardware;

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private String currentMode = "idle";
    private boolean turboActive = false;
    private boolean sniperActive = false;

    private double maxSpeedMps;
    private double maxRotationRps;

    private VisionEstimateResult latestVisionResult;

    // cached pose values (hawk-lib pattern)
    private Pose2d latestOdometryPose = new Pose2d();
    private Pose2d latestFusedPose = new Pose2d();

    // reset tracking for dashboard
    private int resetCount = 0;

    private final OdometryDiagnostics diagnostics;

    // orbit telemetry
    private boolean orbitActive = false;
    private boolean targetLocked = false;
    private double orbitDistanceFeet = 0;
    private double orbitTargetHeadingDeg = 0;
    private double orbitHeadingErrorDeg = 0;
    private double orbitRadialSpeedFps = 0;
    private double orbitTangentSpeedFps = 0;

    // orbit heading control state (for D-term)
    private double previousHeadingError = 0;

    // orbit initial lock state - once locked, stays unlocked
    private boolean orbitInitialLockAcquired = false;

    ChassisSpeeds latestSpeed = Util.ZERO_SPEED;

    /**
     * Creates a {@link SwerveSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public SwerveSubsystem(SwerveHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // initialize odometry at origin
        odometry = new SwerveDriveOdometry(
                KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions(),
                new Pose2d());

        // Field2d for AdvantageScope visualization
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

        // set initial max speeds from config
        updateSpeedLimits();

        // initialize diagnostics
        diagnostics = new OdometryDiagnostics();

        // dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addBooleanProperty("Enabled?", DriverStation::isEnabled, null);
            builder.addBooleanProperty("TurboActive?", () -> turboActive, null);
            builder.addBooleanProperty("SniperActive?", () -> sniperActive, null);
            if (verboseLogging) {
                builder.addDoubleProperty("PoseX", () -> Units.metersToFeet(getPose().getX()), null);
                builder.addDoubleProperty("PoseY", () -> Units.metersToFeet(getPose().getY()), null);
                builder.addDoubleProperty("PoseOmega", () -> getHeading().getDegrees(), null);
                builder.addDoubleProperty("SpeedX", () -> Units.metersToFeet(latestSpeed.vxMetersPerSecond), null);
                builder.addDoubleProperty("SpeedY", () -> Units.metersToFeet(latestSpeed.vyMetersPerSecond), null);
                builder.addDoubleProperty("SpeedOmega", () -> Units.radiansToDegrees(latestSpeed.omegaRadiansPerSecond), null);
            }

            // reset odometry button - click to reset pose and heading to origin
            builder.addBooleanProperty("ResetOdometry", () -> false, (value) -> {
                if (value) {
                    zeroHeading();
                    resetPose(new Pose2d());
                    Util.log("Odometry and heading reset from dashboard");
                }
            });

            // reset counter to verify resets are happening
            builder.addIntegerProperty("ResetCount", () -> resetCount, null);
        });
    }

    /**
     * Updates speed limits from preferences. Call periodically to allow
     * runtime tuning.
     */
    private void updateSpeedLimits() {
        maxSpeedMps = Units.feetToMeters(maxSpeedFps.getAsDouble());
        maxRotationRps = Math.toRadians(maxRotationDps.getAsDouble());
    }

    @Override
    public void periodic() {
        // refresh all hardware signals in a single CAN transaction (reduces CAN bus traffic)
        hardware.refreshSignals();

        // update odometry (standard 50Hz)
        Rotation2d heading = hardware.getHeading();
        SwerveModulePosition[] positions = hardware.getModulePositions();

        odometry.update(heading, positions);
        latestOdometryPose = odometry.getPoseMeters();

        // always update the pose estimator (vision integration happens here)
        poseEstimator.update(heading, positions);
        latestFusedPose = poseEstimator.getEstimatedPosition();

        // update vision pose estimation
        latestVisionResult = LimelightEstimator.updateEstimate(
                poseEstimator,
                heading.getDegrees(),
                hardware.getYawRateDps(),
                poseEstimator.getEstimatedPosition());

        // update diagnostics
        if (enableDiagnostics.getAsBoolean()) {
            diagnostics.update(latestOdometryPose, poseEstimator.getEstimatedPosition(),
                    latestVisionResult);
            diagnostics.publishTelemetry();

            // warn if drift exceeds threshold
            double driftThreshold = driftWarning.getAsDouble();
            if (diagnostics.getCurrentDrift() > driftThreshold) {
                Util.log("WARNING: Odometry drift %.3fm exceeds threshold %.3fm",
                        diagnostics.getCurrentDrift(), driftThreshold);
            }
        }

        // update Field2d for AdvantageScope
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        // publish poses for AdvantageScope
        Util.publishPose("Odometry", latestOdometryPose);
        Util.publishPose("Estimated", poseEstimator.getEstimatedPosition());

        // publish vision pose if valid
        if (latestVisionResult != null && latestVisionResult.accepted && latestVisionResult.getPose() != null) {
            Util.publishPose("Vision", latestVisionResult.getPose());
        }
    }

    /**
     * @return the current robot heading from the gyro
     */
    public Rotation2d getHeading() {
        return hardware.getHeading();
    }

    /**
     * @return the current estimated pose of the robot (fused with vision)
     */
    public Pose2d getPose() {
        return latestFusedPose;
    }

    /**
     * @return the odometry-only pose (no vision correction)
     */
    public Pose2d getOdometryPose() {
        return latestOdometryPose;
    }

    /**
     * Resets the odometry to a specific pose.
     * <p>
     * Uses the current gyro heading and module positions (hawk-lib pattern).
     *
     * @param newPose the pose to reset to
     */
    public void resetPose(Pose2d newPose) {
        Rotation2d currentHeading = hardware.getHeading();
        SwerveModulePosition[] currentPositions = hardware.getModulePositions();

        // reset odometry
        odometry.resetPosition(currentHeading, currentPositions, newPose);
        latestOdometryPose = newPose;

        // reset pose estimator
        poseEstimator.resetPosition(currentHeading, currentPositions, newPose);
        latestFusedPose = newPose;

        // reset vision baseline for jump detection
        LimelightEstimator.resetLastPose();

        // reset diagnostics
        diagnostics.reset();

        // increment counter for dashboard
        resetCount++;
    }

    /**
     * Zeroes the gyro heading.
     */
    public void zeroHeading() {
        hardware.zeroHeading();
    }

    /**
     * Drives the robot using the given speeds.
     *
     * @param speeds        chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param fieldRelative whether speeds are field-relative
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds, fieldRelative, maxSpeedMps);
    }

    /**
     * Drives the robot using the given speeds with a custom max speed limit.
     *
     * @param speeds        chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param fieldRelative whether speeds are field-relative
     * @param maxSpeed      maximum wheel speed for desaturation (m/s)
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double maxSpeed) {
        if (fieldRelative) {
            speeds = Util.fromDriverRelativeSpeeds(speeds, getHeading());
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        hardware.setModuleStates(states);
        latestSpeed = speeds;
    }

    /**
     * Drives with rotation-priority desaturation.
     * <p>
     * When wheel speeds would exceed limits, this method scales down translation
     * to preserve rotation authority. This ensures heading control isn't starved
     * during orbit/face-target modes.
     *
     * @param vx       field-relative X velocity (m/s)
     * @param vy       field-relative Y velocity (m/s)
     * @param omega    rotation rate (rad/s)
     * @param maxSpeed maximum wheel speed for desaturation (m/s)
     */
    private void drivePreserveRotation(double vx, double vy, double omega, double maxSpeed) {
        // convert field-relative to robot-relative
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotSpeeds = Util.fromDriverRelativeSpeeds(fieldSpeeds, getHeading());

        // get module states
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(robotSpeeds);

        // find max wheel speed
        double maxWheelSpeed = 0;
        for (SwerveModuleState state : states) {
            maxWheelSpeed = Math.max(maxWheelSpeed, Math.abs(state.speedMetersPerSecond));
        }

        // if within limits, just drive normally
        if (maxWheelSpeed <= maxSpeed) {
            hardware.setModuleStates(states);
            latestSpeed = robotSpeeds;
            return;
        }

        // calculate how much rotation alone would use
        ChassisSpeeds rotationOnly = new ChassisSpeeds(0, 0, omega);
        SwerveModuleState[] rotationStates = KINEMATICS.toSwerveModuleStates(rotationOnly);
        double maxRotationSpeed = 0;
        for (SwerveModuleState state : rotationStates) {
            maxRotationSpeed = Math.max(maxRotationSpeed, Math.abs(state.speedMetersPerSecond));
        }

        // calculate available headroom for translation
        double availableForTranslation = maxSpeed - maxRotationSpeed;
        if (availableForTranslation <= 0) {
            // rotation alone exceeds max - scale rotation and skip translation
            double scale = maxSpeed / maxRotationSpeed;
            robotSpeeds = new ChassisSpeeds(0, 0, omega * scale);
        } else {
            // scale translation to fit in remaining headroom
            double translationMag = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
            if (translationMag > 0) {
                double scale = Math.min(1.0, availableForTranslation / translationMag);
                robotSpeeds = new ChassisSpeeds(
                    robotSpeeds.vxMetersPerSecond * scale,
                    robotSpeeds.vyMetersPerSecond * scale,
                    omega
                );
            }
        }

        states = KINEMATICS.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        hardware.setModuleStates(states);
        latestSpeed = robotSpeeds;
    }

    /**
     * @return the current chassis speeds
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return KINEMATICS.toChassisSpeeds(hardware.getModuleStates());
    }

    /**
     * Drives the robot using robot-relative speeds.
     * <p>
     * This method is used by PathPlanner for autonomous path following.
     *
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        currentMode = "path-following";

        // Compensate for translational skew during rotation (20ms timestep)
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, Util.DT);

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED_MPS);
        hardware.setModuleStates(states);
        latestSpeed = discretizedSpeeds;
    }

    /**
     * Sets the wheels to an X pattern to prevent movement.
     */
    public void setX() {
        hardware.setX();
    }

    /**
     * @return whether the latest vision estimate was accepted
     */
    public boolean isVisionValid() {
        return latestVisionResult != null && latestVisionResult.accepted;
    }

    /**
     * @return the latest vision estimate result (may be null or rejected)
     */
    public VisionEstimateResult getLatestVisionResult() {
        return latestVisionResult;
    }

    /**
     * @return the odometry diagnostics tracker
     */
    public OdometryDiagnostics getDiagnostics() {
        return diagnostics;
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that holds position (sets X pattern)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            setX();
        });
    }

    /**
     * @return a command that resets all wheel angles to 0 (forward facing)
     */
    public Command resetWheelsCommand() {
        return runOnce(() -> {
            hardware.lockTurnMotors();
            Util.log("Wheels reset to forward");
        });
    }

    /**
     * Creates a teleop drive command.
     *
     * @param xSupplier     forward/backward speed (-1 to 1)
     * @param ySupplier     left/right speed (-1 to 1)
     * @param rotSupplier   rotation speed (-1 to 1)
     * @param fieldRelative whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            boolean fieldRelative) {

        return run(() -> {
            currentMode = fieldRelative ? "teleop-field" : "teleop-robot";
            updateSpeedLimits();

            double vx = xSupplier.getAsDouble() * maxSpeedMps;
            double vy = ySupplier.getAsDouble() * maxSpeedMps;
            double omega = rotSupplier.getAsDouble() * maxRotationRps;

            drive(new ChassisSpeeds(vx, vy, omega), fieldRelative);
        });
    }

    /**
     * Creates a teleop drive command with turbo and sniper speed modes.
     * <p>
     * Speed modes are activated by triggers:
     * <ul>
     *   <li>Sniper (left trigger > 0.5): Slow, precise movement</li>
     *   <li>Turbo (right trigger > 0.5): Fast movement</li>
     *   <li>Sniper takes priority if both triggers are pressed</li>
     * </ul>
     *
     * @param xSupplier      forward/backward speed (-1 to 1)
     * @param ySupplier      left/right speed (-1 to 1)
     * @param rotSupplier    rotation speed (-1 to 1)
     * @param sniperTrigger   sniper mode trigger
     * @param turboTrigger    turbo mode trigger
     * @param fieldRelative  whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            BooleanSupplier sniperTrigger,
            BooleanSupplier turboTrigger,
            boolean fieldRelative) {

        return run(() -> {
            updateSpeedLimits();

            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double rot = rotSupplier.getAsDouble();

            // check trigger thresholds
            sniperActive = sniperTrigger.getAsBoolean();
            turboActive = turboTrigger.getAsBoolean();

            // clamp inputs to [-1, 1]
            x = MathUtil.clamp(x, -1.0, 1.0);
            y = MathUtil.clamp(y, -1.0, 1.0);
            rot = MathUtil.clamp(rot, -1.0, 1.0);

            // convert to velocities
            double vx = x * maxSpeedMps;
            double vy = y * maxSpeedMps;
            double omega = rot * maxRotationRps;

            // track effective max speed for desaturation
            double effectiveMaxSpeed = maxSpeedMps;

            // apply speed modifiers (sniper takes priority)
            if (sniperActive) {
                double sf = sniperFactor.getAsDouble();
                vx *= sf;
                vy *= sf;
                effectiveMaxSpeed *= sf;
                if (applySniperToRotation.getAsBoolean()) {
                    omega *= sf;
                }
                currentMode = fieldRelative ? "teleop-sniper" : "teleop-sniper-robot";
            } else if (turboActive) {
                double tf = turboFactor.getAsDouble();
                vx *= tf;
                vy *= tf;
                effectiveMaxSpeed *= tf;
                currentMode = fieldRelative ? "teleop-turbo" : "teleop-turbo-robot";
            } else {
                currentMode = fieldRelative ? "teleop-field" : "teleop-robot";
            }

            drive(new ChassisSpeeds(vx, vy, omega), fieldRelative, effectiveMaxSpeed);
        });
    }

    /**
     * @return a command that zeroes the gyro heading
     */
    public Command zeroHeadingCommand() {
        return runOnce(() -> {
            zeroHeading();
            Util.log("Gyro zeroed");
        });
    }

    /**
     * @return a command that zeros the heading but keeps the current X/Y position
     */
    public Command resetPoseCommand() {
        return runOnce(() -> {
            // keep current X/Y, just zero the heading
            Pose2d current = getPose();
            Pose2d newPose = new Pose2d(current.getX(), current.getY(), new Rotation2d());

            zeroHeading();
            resetPose(newPose);

            Util.log("Heading reset - X:%.2f Y:%.2f Heading:%.1f",
                    getPose().getX(), getPose().getY(), getHeading().getDegrees());
        });
    }

    /**
     * Creates a command that drives at fixed speeds.
     *
     * @param speeds  the speeds to drive at
     * @param seconds how long to drive
     * @return the command
     */
    public Command fixedSpeedCommand(ChassisSpeeds speeds, double seconds) {
        return run(() -> {
            currentMode = "fixed";
            drive(speeds, false);
        }).withTimeout(seconds);
    }

    /**
     * Creates a drive command that automatically faces the target (Tower) while allowing normal translation.
     * <p>
     * The robot will automatically rotate to face the target center while the driver
     * controls translation with the left stick. Right stick provides rotation trim.
     *
     * @param xSupplier       forward/back speed (-1 to 1)
     * @param ySupplier       left/right strafe speed (-1 to 1)
     * @param rotSupplier     rotation trim input (-1 to 1)
     * @param sniperTrigger   sniper mode trigger
     * @param turboTrigger    turbo mode trigger
     * @return the face-target drive command
     */
    public Command faceTargetDriveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            BooleanSupplier sniperTrigger,
            BooleanSupplier turboTrigger) {

        return startRun(
            () -> {
                currentMode = "face-reef";
            },
            () -> {
                updateSpeedLimits();

                // get target position for current alliance
                Translation2d targetPoint = getTargetPoint();
                Pose2d robotPose = getPose();
                Translation2d robotPos = robotPose.getTranslation();

                // vector from robot to target
                Translation2d toTarget = targetPoint.minus(robotPos);
                double distance = toTarget.getNorm();

                // get driver inputs
                double x = xSupplier.getAsDouble();
                double y = ySupplier.getAsDouble();
                double rotTrim = rotSupplier.getAsDouble();

                // clamp inputs
                x = MathUtil.clamp(x, -1.0, 1.0);
                y = MathUtil.clamp(y, -1.0, 1.0);
                rotTrim = MathUtil.clamp(rotTrim, -1.0, 1.0);

                // convert to velocities
                double vx = x * maxSpeedMps;
                double vy = y * maxSpeedMps;

                // check trigger thresholds
                sniperActive = sniperTrigger.getAsBoolean();
                turboActive = turboTrigger.getAsBoolean();

                // track effective max speed for desaturation
                double effectiveMaxSpeed = maxSpeedMps;

                // apply speed modifiers (sniper takes priority)
                if (sniperActive) {
                    double sf = sniperFactor.getAsDouble();
                    vx *= sf;
                    vy *= sf;
                    effectiveMaxSpeed *= sf;
                    currentMode = "face-reef-sniper";
                } else if (turboActive) {
                    double tf = turboFactor.getAsDouble();
                    vx *= tf;
                    vy *= tf;
                    effectiveMaxSpeed *= tf;
                    currentMode = "face-reef-turbo";
                } else {
                    currentMode = "face-reef";
                }

                // heading control: always face the target point
                double targetHeadingDeg = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
                double currentHeadingDeg = robotPose.getRotation().getDegrees();
                double headingError = Util.degreeModulus(targetHeadingDeg - currentHeadingDeg);

                // update telemetry
                orbitDistanceFeet = Units.metersToFeet(distance);
                orbitTargetHeadingDeg = targetHeadingDeg;
                orbitHeadingErrorDeg = headingError;
                targetLocked = Math.abs(headingError) < 2.0;

                // calculate rotation speed - proportional with saturation to max
                double maxOmegaDps = Math.toDegrees(maxRotationRps);
                double omegaDps = headingKP.getAsDouble() * headingError;
                omegaDps = MathUtil.clamp(omegaDps, -maxOmegaDps, maxOmegaDps);

                // add manual trim
                double trimSpeed = rotTrim * maxRotationRps * 0.3;
                double omega = Math.toRadians(omegaDps) + trimSpeed;

                // drive with rotation preserved - translation scales down if needed
                drivePreserveRotation(vx, vy, omega, effectiveMaxSpeed);
            }
        );
    }

    /**
     * @return the target point (Tower position for 2026) for the current alliance
     */
    public Translation2d getTargetPoint() {
        if (Util.isRedAlliance()) {
            return new Translation2d(redTargetX.getAsDouble(), redTargetY.getAsDouble());
        } else {
            return new Translation2d(blueTargetX.getAsDouble(), blueTargetY.getAsDouble());
        }
    }

    /**
     * Creates an orbit drive command that orbits around the target (Tower) while facing it.
     * <p>
     * The left stick Y controls radial movement (toward/away from target).
     * The left stick X controls tangential movement (orbiting around target).
     * The right stick X provides fine rotation adjustment.
     * <p>
     * Turbo and sniper speed modifiers are applied to the orbit speeds.
     *
     * @param radialSupplier   radial input (-1 to 1), positive = toward target
     * @param tangentSupplier  tangent input (-1 to 1), positive = CCW orbit
     * @param rotSupplier      rotation trim input (-1 to 1)
     * @param sniperTrigger    sniper mode trigger
     * @param turboTrigger     turbo mode trigger
     * @return the orbit command
     */
    public Command orbitCommand(
            DoubleSupplier radialSupplier,
            DoubleSupplier tangentSupplier,
            DoubleSupplier rotSupplier,
            BooleanSupplier sniperTrigger,
            BooleanSupplier turboTrigger) {

        return startRun(
            () -> {
                currentMode = "orbit-locking";
                orbitActive = true;
                previousHeadingError = 0;
                orbitInitialLockAcquired = false;
            },
            () -> {
                updateSpeedLimits();

                // get orbit point for current alliance
                Translation2d orbitPoint = getTargetPoint();
                Pose2d robotPose = getPose();
                Translation2d robotPos = robotPose.getTranslation();

                // vector from robot to orbit point
                Translation2d toOrbit = orbitPoint.minus(robotPos);
                double distance = toOrbit.getNorm();

                // prevent singularity when too close to orbit point
                double minDist = minDistance.getAsDouble();
                if (distance < minDist) {
                    // just stop if we're too close
                    drive(Util.ZERO_SPEED, false);
                    return;
                }

                // unit vectors: radial (toward orbit point) and tangent (perpendicular, CCW)
                Translation2d radialUnit = toOrbit.div(distance);
                Translation2d tangentUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

                // get driver inputs
                double radialInput = radialSupplier.getAsDouble();
                double tangentInput = tangentSupplier.getAsDouble();
                double rotInput = rotSupplier.getAsDouble();

                // clamp inputs
                radialInput = MathUtil.clamp(radialInput, -1.0, 1.0);
                tangentInput = MathUtil.clamp(tangentInput, -1.0, 1.0);
                rotInput = MathUtil.clamp(rotInput, -1.0, 1.0);

                // convert to velocities (in meters per second)
                double radialSpeed = radialInput * Units.feetToMeters(maxRadialSpeedFps.getAsDouble());
                double tangentSpeed = tangentInput * Units.feetToMeters(maxTangentSpeedFps.getAsDouble());

                // check trigger thresholds
                sniperActive = sniperTrigger.getAsBoolean();
                turboActive = turboTrigger.getAsBoolean();

                // apply speed modifiers (sniper takes priority)
                if (sniperActive) {
                    double sf = sniperFactor.getAsDouble();
                    radialSpeed *= sf;
                    tangentSpeed *= sf;
                } else if (turboActive) {
                    // turbo uses same multiplier as regular driving
                    double tf = turboFactor.getAsDouble();
                    radialSpeed *= tf;
                    tangentSpeed *= tf;
                }

                // heading control: always face the orbit point
                double targetHeadingDeg = Math.toDegrees(Math.atan2(toOrbit.getY(), toOrbit.getX()));
                double currentHeadingDeg = robotPose.getRotation().getDegrees();
                double headingError = Util.degreeModulus(targetHeadingDeg - currentHeadingDeg);

                // check if we're locked on target (only matters on initial activation)
                // once locked, stays unlocked for the rest of the command
                if (!orbitInitialLockAcquired && Math.abs(headingError) < lockThreshold.getAsDouble()) {
                    orbitInitialLockAcquired = true;
                }
                boolean canMove = orbitInitialLockAcquired;

                // update mode based on lock state and speed mode
                if (!orbitInitialLockAcquired) {
                    currentMode = "orbit-locking";
                } else if (sniperActive) {
                    currentMode = "orbit-sniper";
                } else if (turboActive) {
                    currentMode = "orbit-turbo";
                } else {
                    currentMode = "orbit";
                }

                // combine radial and tangent into field-relative velocity
                // suppress translation until locked on target
                double vxField = 0;
                double vyField = 0;
                if (canMove) {
                    vxField = radialUnit.getX() * radialSpeed + tangentUnit.getX() * tangentSpeed;
                    vyField = radialUnit.getY() * radialSpeed + tangentUnit.getY() * tangentSpeed;
                }

                // update telemetry
                orbitDistanceFeet = Units.metersToFeet(distance);
                orbitTargetHeadingDeg = targetHeadingDeg;
                orbitHeadingErrorDeg = headingError;
                orbitRadialSpeedFps = canMove ? Units.metersToFeet(radialSpeed) : 0;
                orbitTangentSpeedFps = canMove ? Units.metersToFeet(tangentSpeed) : 0;
                targetLocked = Math.abs(headingError) < headingTolerance.getAsDouble();

                // feedforward: predict rotation rate needed for tangential movement
                // only apply when moving (canMove), otherwise just use PD to lock on target
                double feedforwardDps = canMove ? Math.toDegrees(tangentSpeed / distance) : 0;

                // calculate rotation with PD control and saturation to max
                double maxOmegaDps = Math.toDegrees(maxRotationRps);
                double omegaDps = feedforwardDps + headingKP.getAsDouble() * headingError;

                // add D-term: rate of error change (degrees per second)
                double errorRate = (headingError - previousHeadingError) / Util.DT;
                double dTermDps = headingKD.getAsDouble() * errorRate;
                omegaDps += dTermDps;
                previousHeadingError = headingError;

                omegaDps = MathUtil.clamp(omegaDps, -maxOmegaDps, maxOmegaDps);

                // add manual trim
                double rotTrim = rotInput * maxRotationRps * rotationTrimAuthority.getAsDouble();
                double omega = Math.toRadians(omegaDps) + rotTrim;

                // drive with rotation preserved - translation scales down if needed
                drivePreserveRotation(vxField, vyField, omega, maxSpeedMps);
            }
        ).finallyDo(() -> orbitActive = false);
    }

//endregion
}
