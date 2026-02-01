package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * Standalone testbot for System Identification (SysId) of swerve drive motors.
 * <p>
 * Uses CTRE's built-in SysId routines from CommandSwerveDrivetrain.
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
 *   <tr><td>Start</td><td>Reset pose</td></tr>
 * </table>
 * <p>
 * Run with: {@code ./gradlew deploy -Probot=SysIdTestbot}
 * <p>
 * For development/structure testing: {@code ./gradlew simulateJava -Probot=SysIdTestbot}
 */
public class SysIdTestbot extends TimedRobot {

    private CommandSwerveDrivetrain drivetrain;
    private GameController controller;
    private String currentTest = "Idle";

    // track last 2 buttons pressed
    private String lastButton = "(none)";
    private String previousButton = "(none)";

    // odometry tracking
    private final Field2d field2d = new Field2d();

    // AdvantageScope pose publisher
    private StructPublisher<Pose2d> posePublisher;

    @Override
    public void robotInit() {
        System.out.println(">>> SysIdTestbot starting...");
        System.out.println(">>> WARNING: SysId requires REAL HARDWARE for useful data!");
        System.out.println(">>> Run on a flat surface with ample space for robot movement.");

        // create CTRE drivetrain (handles sim/real automatically)
        drivetrain = TunerConstants.createDrivetrain();
        controller = new GameController(0);

        // publish field2d to dashboard
        SmartDashboard.putData("Field", field2d);

        // create AdvantageScope-compatible pose publisher
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SysId/Pose", Pose2d.struct).publish();

        // manual button bindings using CTRE's built-in SysId routines
        controller.a().whileTrue(
            logButton("A-QuasiForward")
                .andThen(Commands.runOnce(() -> currentTest = "Quasistatic Forward"))
                .andThen(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        );
        controller.b().whileTrue(
            logButton("B-QuasiReverse")
                .andThen(Commands.runOnce(() -> currentTest = "Quasistatic Reverse"))
                .andThen(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        );
        controller.x().whileTrue(
            logButton("X-DynForward")
                .andThen(Commands.runOnce(() -> currentTest = "Dynamic Forward"))
                .andThen(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        );
        controller.y().whileTrue(
            logButton("Y-DynReverse")
                .andThen(Commands.runOnce(() -> currentTest = "Dynamic Reverse"))
                .andThen(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        );

        // reset pose
        controller.start().onTrue(logButton("Start-Reset").andThen(Commands.runOnce(() -> {
            drivetrain.resetPose(Pose2d.kZero);
            System.out.println(">>> Pose reset");
        })));

        // dashboard telemetry - SysId status
        SmartDashboard.putData("SysId", builder -> {
            builder.addStringProperty("CurrentTest", () -> currentTest, null);
        });

        // dashboard telemetry - button tracking
        SmartDashboard.putData("ButtonLog", builder -> {
            builder.addStringProperty("Last", () -> lastButton, null);
            builder.addStringProperty("Previous", () -> previousButton, null);
        });

        // dashboard telemetry - controller inputs
        SmartDashboard.putData("Controller", builder -> {
            builder.addBooleanProperty("A", () -> controller.a().getAsBoolean(), null);
            builder.addBooleanProperty("B", () -> controller.b().getAsBoolean(), null);
            builder.addBooleanProperty("X", () -> controller.x().getAsBoolean(), null);
            builder.addBooleanProperty("Y", () -> controller.y().getAsBoolean(), null);
            builder.addBooleanProperty("Start", () -> controller.start().getAsBoolean(), null);
            builder.addBooleanProperty("Back", () -> controller.back().getAsBoolean(), null);
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

    /** Timeout for quasistatic tests (seconds) */
    private static final double QUASISTATIC_TIMEOUT = 4.0;

    /** Timeout for dynamic tests (seconds) */
    private static final double DYNAMIC_TIMEOUT = 2.0;

    /** Pause between test runs (seconds) */
    private static final double PAUSE_BETWEEN_RUNS = 1.5;

    /**
     * Creates a command that runs all SysId tests in sequence automatically.
     */
    private Command autoSysIdSequence() {
        return Commands.sequence(
            // reset pose before starting
            Commands.runOnce(() -> {
                drivetrain.resetPose(Pose2d.kZero);
                System.out.println(">>> Starting SysId sequence");
            }),

            // Quasistatic tests
            Commands.runOnce(() -> currentTest = "Quasi Forward"),
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(QUASISTATIC_TIMEOUT),
            Commands.waitSeconds(PAUSE_BETWEEN_RUNS),

            Commands.runOnce(() -> currentTest = "Quasi Reverse"),
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(QUASISTATIC_TIMEOUT),
            Commands.waitSeconds(PAUSE_BETWEEN_RUNS),

            // Dynamic tests
            Commands.runOnce(() -> currentTest = "Dynamic Forward"),
            drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(DYNAMIC_TIMEOUT),
            Commands.waitSeconds(PAUSE_BETWEEN_RUNS),

            Commands.runOnce(() -> currentTest = "Dynamic Reverse"),
            drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(DYNAMIC_TIMEOUT),

            // complete
            Commands.runOnce(() -> {
                currentTest = "Complete!";
                System.out.println(">>> SysId Complete!");
                System.out.println(">>> Export log data from the robot.");
            })
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // get current pose from drivetrain
        Pose2d currentPose = drivetrain.getState().Pose;

        // update field visualization
        field2d.setRobotPose(currentPose);

        // publish pose for AdvantageScope
        posePublisher.set(currentPose);

        // publish pose to SmartDashboard
        SmartDashboard.putNumber("Swerve/Pose X (ft)", Units.metersToFeet(currentPose.getX()));
        SmartDashboard.putNumber("Swerve/Pose Y (ft)", Units.metersToFeet(currentPose.getY()));
        SmartDashboard.putNumber("Swerve/Pose Heading (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putString("Swerve/Test", currentTest);
    }

    @Override
    public void autonomousInit() {
        System.out.println(">>> Autonomous enabled - starting SysId sequence");
        CommandScheduler.getInstance().schedule(autoSysIdSequence());
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        currentTest = "Manual Mode";
        System.out.println(">>> Teleop enabled - use A/B/X/Y for manual SysId control");
    }

    @Override
    public void disabledInit() {
        currentTest = "Disabled";
    }
}
