package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveHardware;

import static edu.wpi.first.units.Units.*;

/**
 * Standalone testbot for System Identification (SysId) of swerve drive motors.
 * <p>
 * This testbot characterizes the drive motors to determine accurate feedforward
 * constants (kS, kV, kA) using WPILib's SysIdRoutine class.
 * <p>
 * <b>IMPORTANT:</b> SysId requires real hardware - simulation data is not useful
 * for characterization. Run this on the actual robot on a flat surface with
 * ample space for the robot to move.
 * <p>
 * <b>Automatic Mode (Recommended):</b>
 * <ol>
 *   <li>Deploy to robot: {@code ./gradlew deploy -Probot=SysIdTestbot}</li>
 *   <li>Enable Autonomous mode in Driver Station</li>
 *   <li>All 4 tests run automatically in sequence (~30 seconds total)</li>
 *   <li>Tests complete with "SysId Complete!" message</li>
 * </ol>
 * <p>
 * <b>Manual Mode (Backup):</b>
 * <table>
 *   <tr><th>Button</th><th>Action</th></tr>
 *   <tr><td>A (hold)</td><td>Quasistatic Forward</td></tr>
 *   <tr><td>B (hold)</td><td>Quasistatic Reverse</td></tr>
 *   <tr><td>X (hold)</td><td>Dynamic Forward</td></tr>
 *   <tr><td>Y (hold)</td><td>Dynamic Reverse</td></tr>
 *   <tr><td>Start</td><td>Reset encoders</td></tr>
 * </table>
 * <p>
 * After running tests, export the log data and load into WPILib's SysId analysis
 * tool to calculate kS, kV, kA values.
 * <p>
 * Run with: {@code ./gradlew deploy -Probot=SysIdTestbot}
 * <p>
 * For development/structure testing: {@code ./gradlew simulateJava -Probot=SysIdTestbot}
 */
public class SysIdTestbot extends TimedRobot {

    //==========================================================================
    // Safety limits - adjust these based on your testing area!
    // Configured for MK4i L3 with Kraken X60 (max ~5.6 m/s theoretical)
    //==========================================================================

    /** Maximum distance from origin before emergency stop (meters) */
    private static final double MAX_DISTANCE_METERS = 4.0;  // ~13 feet

    /** Maximum velocity before emergency stop (m/s) - Kraken X60 + MK4i L3 max is ~5.6 m/s */
    private static final double MAX_VELOCITY_MPS = 6.0;  // ~20 ft/s (above theoretical max)

    /** Maximum rotation from starting heading before emergency stop (degrees) */
    private static final double MAX_ROTATION_DEGREES = 30.0;  // tighter limit - wheels should stay straight

    //==========================================================================

    private SwerveHardware hardware;
    private SysIdRoutine sysIdRoutine;
    private CommandXboxController controller;
    private double appliedVoltage = 0;
    private String currentTest = "Idle";

    // track last 2 buttons pressed
    private String lastButton = "(none)";
    private String previousButton = "(none)";

    // odometry tracking
    private SwerveDriveOdometry odometry;
    private final Field2d field2d = new Field2d();
    private Pose2d currentPose = new Pose2d();

    // AdvantageScope pose publisher
    private StructPublisher<Pose2d> posePublisher;

    // safety tracking
    private boolean safetyTriggered = false;
    private String safetyReason = "";

    // empty subsystem for SysIdRoutine requirements
    private final Subsystem dummySubsystem = new Subsystem() {};

    @Override
    public void robotInit() {
        System.out.println(">>> SysIdTestbot starting...");
        System.out.println(">>> WARNING: SysId requires REAL HARDWARE for useful data!");
        System.out.println(">>> Run on a flat surface with ample space for robot movement.");

        // use appropriate hardware based on environment
        hardware = isSimulation() ? new SwerveHardwareSim() : new SwerveHardwareCTRE();
        controller = new CommandXboxController(0);

        // initialize odometry
        hardware.refreshSignals();
        odometry = new SwerveDriveOdometry(
            Config.Swerve.KINEMATICS,
            hardware.getHeading(),
            hardware.getModulePositions()
        );

        // publish field2d to dashboard
        SmartDashboard.putData("Field", field2d);

        // create AdvantageScope-compatible pose publisher
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SysId/Pose", Pose2d.struct).publish();

        // create SysId routine for drive motors
        // slower ramp rate = robot travels less distance per test
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),  // 0.5 V/s ramp rate for quasistatic (slower = less distance)
                Volts.of(3),                // 3V step voltage for dynamic (lower = safer)
                Seconds.of(10)              // 10s timeout per test
            ),
            new SysIdRoutine.Mechanism(
                this::voltageDrive,
                this::logMotors,
                dummySubsystem
            )
        );

        // manual button bindings (backup if autonomous doesn't work)
        controller.a().whileTrue(
            logButton("A-QuasiForward")
                .andThen(Commands.runOnce(() -> currentTest = "Quasistatic Forward"))
                .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        );
        controller.b().whileTrue(
            logButton("B-QuasiReverse")
                .andThen(Commands.runOnce(() -> currentTest = "Quasistatic Reverse"))
                .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
        );
        controller.x().whileTrue(
            logButton("X-DynForward")
                .andThen(Commands.runOnce(() -> currentTest = "Dynamic Forward"))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
        );
        controller.y().whileTrue(
            logButton("Y-DynReverse")
                .andThen(Commands.runOnce(() -> currentTest = "Dynamic Reverse"))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
        );

        // reset encoders, odometry, and safety
        controller.start().onTrue(logButton("Start-Reset").andThen(Commands.runOnce(() -> {
            hardware.resetEncoders();
            hardware.zeroHeading();
            odometry.resetPosition(
                hardware.getHeading(),
                hardware.getModulePositions(),
                new Pose2d()
            );
            currentPose = new Pose2d();
            resetSafety();
            System.out.println(">>> Encoders, odometry, and safety reset");
        })));

        // reset gyro only
        controller.back().onTrue(logButton("Back-ZeroGyro").andThen(Commands.runOnce(() -> {
            hardware.zeroHeading();
            System.out.println(">>> Gyro zeroed");
        })));

        // dashboard telemetry - SysId status
        SmartDashboard.putData("SysId", builder -> {
            builder.addStringProperty("CurrentTest", () -> currentTest, null);
            builder.addDoubleProperty("AppliedVolts", () -> appliedVoltage, null);
            builder.addDoubleProperty("PositionMeters", hardware::getAverageDrivePositionMeters, null);
            builder.addDoubleProperty("VelocityMps", hardware::getAverageDriveVelocityMps, null);
        });

        // dashboard telemetry - safety status
        SmartDashboard.putData("Safety", builder -> {
            builder.addBooleanProperty("TRIGGERED", () -> safetyTriggered, null);
            builder.addStringProperty("Reason", () -> safetyReason, null);
            builder.addDoubleProperty("DistFromOrigin (m)", this::getDistanceFromOrigin, null);
            builder.addDoubleProperty("MaxDist (m)", () -> MAX_DISTANCE_METERS, null);
            builder.addDoubleProperty("MaxVel (mps)", () -> MAX_VELOCITY_MPS, null);
            builder.addDoubleProperty("MaxRot (deg)", () -> MAX_ROTATION_DEGREES, null);
        });

        // dashboard telemetry - button tracking
        SmartDashboard.putData("ButtonLog", builder -> {
            builder.addStringProperty("Last", () -> lastButton, null);
            builder.addStringProperty("Previous", () -> previousButton, null);
        });

        // dashboard telemetry - controller inputs
        SmartDashboard.putData("Controller", builder -> {
            builder.addBooleanProperty("A", () -> controller.getHID().getAButton(), null);
            builder.addBooleanProperty("B", () -> controller.getHID().getBButton(), null);
            builder.addBooleanProperty("X", () -> controller.getHID().getXButton(), null);
            builder.addBooleanProperty("Y", () -> controller.getHID().getYButton(), null);
            builder.addBooleanProperty("Start", () -> controller.getHID().getStartButton(), null);
            builder.addBooleanProperty("Back", () -> controller.getHID().getBackButton(), null);
            builder.addBooleanProperty("LeftBumper", () -> controller.getHID().getLeftBumper(), null);
            builder.addBooleanProperty("RightBumper", () -> controller.getHID().getRightBumper(), null);
            builder.addDoubleProperty("LeftX", () -> controller.getHID().getLeftX(), null);
            builder.addDoubleProperty("LeftY", () -> controller.getHID().getLeftY(), null);
            builder.addDoubleProperty("RightX", () -> controller.getHID().getRightX(), null);
            builder.addDoubleProperty("RightY", () -> controller.getHID().getRightY(), null);
        });

        // dashboard telemetry - odometry/pose (in feet and degrees for readability)
        SmartDashboard.putData("Pose", builder -> {
            builder.addDoubleProperty("X (ft)", () -> Units.metersToFeet(currentPose.getX()), null);
            builder.addDoubleProperty("Y (ft)", () -> Units.metersToFeet(currentPose.getY()), null);
            builder.addDoubleProperty("Heading (deg)", () -> currentPose.getRotation().getDegrees(), null);
            builder.addDoubleProperty("Gyro (deg)", () -> hardware.getHeading().getDegrees(), null);
            builder.addDoubleProperty("YawRate (dps)", hardware::getYawRateDps, null);
        });

        // dashboard telemetry - module states
        SmartDashboard.putData("Modules", builder -> {
            SwerveModuleState[] states = hardware.getModuleStates();
            SwerveModulePosition[] positions = hardware.getModulePositions();
            // FL module
            builder.addDoubleProperty("FL Velocity (mps)", () -> states[0].speedMetersPerSecond, null);
            builder.addDoubleProperty("FL Angle (deg)", () -> states[0].angle.getDegrees(), null);
            builder.addDoubleProperty("FL Position (m)", () -> positions[0].distanceMeters, null);
            // FR module
            builder.addDoubleProperty("FR Velocity (mps)", () -> states[1].speedMetersPerSecond, null);
            builder.addDoubleProperty("FR Angle (deg)", () -> states[1].angle.getDegrees(), null);
            builder.addDoubleProperty("FR Position (m)", () -> positions[1].distanceMeters, null);
            // BL module
            builder.addDoubleProperty("BL Velocity (mps)", () -> states[2].speedMetersPerSecond, null);
            builder.addDoubleProperty("BL Angle (deg)", () -> states[2].angle.getDegrees(), null);
            builder.addDoubleProperty("BL Position (m)", () -> positions[2].distanceMeters, null);
            // BR module
            builder.addDoubleProperty("BR Velocity (mps)", () -> states[3].speedMetersPerSecond, null);
            builder.addDoubleProperty("BR Angle (deg)", () -> states[3].angle.getDegrees(), null);
            builder.addDoubleProperty("BR Position (m)", () -> positions[3].distanceMeters, null);
        });

        System.out.println(">>> SysIdTestbot initialized");
        System.out.println(">>> Enable AUTONOMOUS for automatic test sequence");
        System.out.println(">>> Or use A/B/X/Y buttons for manual control");
    }

    /**
     * Creates a command that logs a button press to console and dashboard.
     */
    private Command logButton(String name) {
        return Commands.runOnce(() -> {
            previousButton = lastButton;
            lastButton = name;
            System.out.println(">>> BUTTON PRESSED: " + name);
        });
    }

    //==========================================================================
    // Safety methods
    //==========================================================================

    /** @return distance from origin in meters */
    private double getDistanceFromOrigin() {
        return Math.hypot(currentPose.getX(), currentPose.getY());
    }

    /** Checks safety limits and triggers emergency stop if exceeded */
    private void checkSafetyLimits() {
        if (safetyTriggered) {
            return;  // already triggered, don't spam console
        }

        double distance = getDistanceFromOrigin();
        double velocity = Math.abs(hardware.getAverageDriveVelocityMps());
        double rotation = Math.abs(currentPose.getRotation().getDegrees());

        if (distance > MAX_DISTANCE_METERS) {
            triggerSafety("DISTANCE EXCEEDED: " + String.format("%.2f", distance) + "m > " + MAX_DISTANCE_METERS + "m");
        } else if (velocity > MAX_VELOCITY_MPS) {
            triggerSafety("VELOCITY EXCEEDED: " + String.format("%.2f", velocity) + "mps > " + MAX_VELOCITY_MPS + "mps");
        } else if (rotation > MAX_ROTATION_DEGREES) {
            triggerSafety("ROTATION EXCEEDED: " + String.format("%.1f", rotation) + "deg > " + MAX_ROTATION_DEGREES + "deg");
        }
    }

    /** Triggers emergency stop */
    private void triggerSafety(String reason) {
        safetyTriggered = true;
        safetyReason = reason;
        currentTest = "SAFETY STOP";
        appliedVoltage = 0;
        hardware.setDriveVoltage(0);

        // cancel all running commands
        CommandScheduler.getInstance().cancelAll();

        System.out.println(">>> !!! SAFETY TRIGGERED !!!");
        System.out.println(">>> " + reason);
        System.out.println(">>> Press START to reset and continue");
    }

    /** Resets safety state (called when encoders are reset) */
    private void resetSafety() {
        safetyTriggered = false;
        safetyReason = "";
        System.out.println(">>> Safety reset");
    }

    /** Number of times to run each test type for better data */
    private static final int RUNS_PER_TEST = 3;

    /** Timeout for quasistatic tests (seconds) - shorter since ramp is slower */
    private static final double QUASISTATIC_TIMEOUT = 4.0;

    /** Timeout for dynamic tests (seconds) */
    private static final double DYNAMIC_TIMEOUT = 2.0;

    /** Pause between test runs (seconds) */
    private static final double PAUSE_BETWEEN_RUNS = 1.5;

    /**
     * Creates a command that runs all SysId tests in sequence automatically.
     * Runs each test type multiple times (forward/reverse alternating) for better data.
     */
    private Command autoSysIdSequence() {
        return Commands.sequence(
            // reset encoders before starting
            Commands.runOnce(() -> {
                hardware.resetEncoders();
                hardware.zeroHeading();
                resetSafety();
                System.out.println(">>> Starting SysId sequence - " + RUNS_PER_TEST + " runs per test type");
                System.out.println(">>> Quasistatic: " + QUASISTATIC_TIMEOUT + "s, Dynamic: " + DYNAMIC_TIMEOUT + "s");
            }),

            // Quasistatic tests (multiple runs, alternating direction)
            Commands.runOnce(() -> System.out.println(">>> === QUASISTATIC TESTS ===")),
            quasistaticRuns(),

            // Brief pause between test types
            Commands.runOnce(() -> currentTest = "Switching to Dynamic..."),
            Commands.waitSeconds(2.0),

            // Dynamic tests (multiple runs, alternating direction)
            Commands.runOnce(() -> System.out.println(">>> === DYNAMIC TESTS ===")),
            dynamicRuns(),

            // complete
            Commands.runOnce(() -> {
                currentTest = "Complete!";
                appliedVoltage = 0;
                System.out.println(">>> ============================");
                System.out.println(">>> SysId Complete!");
                System.out.println(">>> Total runs: " + (RUNS_PER_TEST * 4) + " (" + RUNS_PER_TEST + " each direction, 2 test types)");
                System.out.println(">>> Export log data from the robot.");
                System.out.println(">>> ============================");
            })
        );
    }

    /** Creates quasistatic test runs (alternating forward/reverse) */
    private Command quasistaticRuns() {
        Command[] runs = new Command[RUNS_PER_TEST * 2];
        for (int i = 0; i < RUNS_PER_TEST; i++) {
            int runNum = i + 1;
            // Forward run
            runs[i * 2] = Commands.sequence(
                Commands.runOnce(() -> {
                    currentTest = "Quasi Forward " + runNum + "/" + RUNS_PER_TEST;
                    System.out.println(">>> Quasistatic Forward (run " + runNum + "/" + RUNS_PER_TEST + ")");
                }),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(QUASISTATIC_TIMEOUT),
                Commands.runOnce(() -> currentTest = "Pausing..."),
                Commands.waitSeconds(PAUSE_BETWEEN_RUNS)
            );
            // Reverse run
            runs[i * 2 + 1] = Commands.sequence(
                Commands.runOnce(() -> {
                    currentTest = "Quasi Reverse " + runNum + "/" + RUNS_PER_TEST;
                    System.out.println(">>> Quasistatic Reverse (run " + runNum + "/" + RUNS_PER_TEST + ")");
                }),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(QUASISTATIC_TIMEOUT),
                Commands.runOnce(() -> currentTest = "Pausing..."),
                Commands.waitSeconds(PAUSE_BETWEEN_RUNS)
            );
        }
        return Commands.sequence(runs);
    }

    /** Creates dynamic test runs (alternating forward/reverse) */
    private Command dynamicRuns() {
        Command[] runs = new Command[RUNS_PER_TEST * 2];
        for (int i = 0; i < RUNS_PER_TEST; i++) {
            int runNum = i + 1;
            // Forward run
            runs[i * 2] = Commands.sequence(
                Commands.runOnce(() -> {
                    currentTest = "Dynamic Forward " + runNum + "/" + RUNS_PER_TEST;
                    System.out.println(">>> Dynamic Forward (run " + runNum + "/" + RUNS_PER_TEST + ")");
                }),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(DYNAMIC_TIMEOUT),
                Commands.runOnce(() -> currentTest = "Pausing..."),
                Commands.waitSeconds(PAUSE_BETWEEN_RUNS)
            );
            // Reverse run
            runs[i * 2 + 1] = Commands.sequence(
                Commands.runOnce(() -> {
                    currentTest = "Dynamic Reverse " + runNum + "/" + RUNS_PER_TEST;
                    System.out.println(">>> Dynamic Reverse (run " + runNum + "/" + RUNS_PER_TEST + ")");
                }),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(DYNAMIC_TIMEOUT),
                Commands.runOnce(() -> currentTest = "Pausing..."),
                Commands.waitSeconds(PAUSE_BETWEEN_RUNS)
            );
        }
        return Commands.sequence(runs);
    }

    /**
     * Applies voltage to drive motors while keeping wheels pointed forward.
     * Will not apply voltage if safety has been triggered.
     */
    private void voltageDrive(Voltage voltage) {
        // don't apply voltage if safety triggered
        if (safetyTriggered) {
            appliedVoltage = 0;
            hardware.setDriveVoltage(0);
            return;
        }

        appliedVoltage = voltage.in(Volts);
        hardware.lockTurnMotors();  // keep wheels pointing forward
        hardware.setDriveVoltage(appliedVoltage);

        // print voltage every ~0.5 seconds (every 25 cycles at 50Hz)
        if (Math.abs(appliedVoltage) > 0.01 && System.currentTimeMillis() % 500 < 20) {
            System.out.printf(">>> Voltage: %.2fV | Velocity: %.2f m/s | Position: %.2fm%n",
                appliedVoltage,
                hardware.getAverageDriveVelocityMps(),
                hardware.getAverageDrivePositionMeters());
        }
    }

    /**
     * Logs motor data for SysId analysis.
     */
    private void logMotors(SysIdRoutineLog log) {
        hardware.refreshSignals();
        log.motor("drive")
            .voltage(Volts.of(appliedVoltage))
            .linearPosition(Meters.of(hardware.getAverageDrivePositionMeters()))
            .linearVelocity(MetersPerSecond.of(hardware.getAverageDriveVelocityMps()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // update odometry
        hardware.refreshSignals();
        currentPose = odometry.update(
            hardware.getHeading(),
            hardware.getModulePositions()
        );

        // update field visualization
        field2d.setRobotPose(currentPose);

        // publish pose for AdvantageScope (struct format)
        posePublisher.set(currentPose);

        // check safety limits
        checkSafetyLimits();

        // publish pose directly to SmartDashboard for Elastic/Shuffleboard compatibility
        SmartDashboard.putNumber("Swerve/Pose X (ft)", Units.metersToFeet(currentPose.getX()));
        SmartDashboard.putNumber("Swerve/Pose Y (ft)", Units.metersToFeet(currentPose.getY()));
        SmartDashboard.putNumber("Swerve/Pose Heading (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro (deg)", hardware.getHeading().getDegrees());
        SmartDashboard.putNumber("Swerve/Velocity (mps)", hardware.getAverageDriveVelocityMps());
        SmartDashboard.putNumber("Swerve/Position (m)", hardware.getAverageDrivePositionMeters());
        SmartDashboard.putNumber("Swerve/Applied Volts", appliedVoltage);
        SmartDashboard.putString("Swerve/Test", currentTest);
        SmartDashboard.putBoolean("Swerve/Safety Triggered", safetyTriggered);
    }

    @Override
    public void autonomousInit() {
        System.out.println(">>> Autonomous enabled - starting SysId sequence");
        autoSysIdSequence().schedule();
    }

    @Override
    public void teleopInit() {
        // cancel any running autonomous commands
        CommandScheduler.getInstance().cancelAll();
        currentTest = "Manual Mode";
        appliedVoltage = 0;
        System.out.println(">>> Teleop enabled - use A/B/X/Y for manual SysId control");
    }

    @Override
    public void disabledInit() {
        currentTest = "Disabled";
        appliedVoltage = 0;
    }
}
