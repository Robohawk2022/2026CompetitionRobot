package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.DigitBoardProgramPicker;
import frc.robot.util.Util;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;

import static frc.robot.Config.PathPlanner.debugLogging;
import static frc.robot.Config.PathPlanner.maxSpeed;
import static frc.robot.Config.PathPlanner.rotationP;
import static frc.robot.Config.PathPlanner.translationP;
import static frc.robot.Config.PathPlanner.unloadSecs;

/**
 * Subsystem that manages a set of declared autonomous programs and allows
 * selecting one to run.
 */
public class AutonomousSubsystem extends SubsystemBase {

//region Implementation --------------------------------------------------------

    final SwerveSubsystem swerve;
    final ShooterSubsystem shooter;
    final BallPathSubsystem ballPath;
    final LEDSubsystem led;
    final DigitBoardProgramPicker picker;
    String selected;
    Command command;

    public AutonomousSubsystem(SwerveSubsystem swerve,
                               ShooterSubsystem shooter,
                               BallPathSubsystem ballPath,
                               LEDSubsystem led) {

        this.swerve = Objects.requireNonNull(swerve);
        this.shooter = Objects.requireNonNull(shooter);
        this.ballPath = Objects.requireNonNull(ballPath);
        this.led = Objects.requireNonNull(led);
        this.picker = new DigitBoardProgramPicker(
                "Auto Program",
                getProgramNames());
        this.selected = picker.get();

        // there's a lot happening under the covers here, and PathPlanner
        // might generate an error if there's a problem with config files,
        // or calculating a path, or something like that.
        //
        // we don't want the entire robot to crash if there's a problem
        // with autonomous, so we wrap this whole thing with an exception
        // handler. if there is an error, we will log it and then use our
        // emergency backup configuration

        configureAutoBuilder();

        if (debugLogging.getAsBoolean()) {
            configureLogging();
        }

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Program", () -> selected, null);
            builder.addBooleanProperty("Running?", () -> command != null && command.isScheduled(), null);
        });
    }

    /**
     * Configure PathPlanner's command builder
     */
    private void configureAutoBuilder() {

        // this loads the settings for the robot from PathPlanner's GUI settings file
        Util.log("[auto] loading robot configuration from PathPlanner settings");

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Util.log("[auto] failed to load config from GUI, using manual config: %s", e.getMessage());
            config = new RobotConfig(
                    SwerveHardwareConfig.ROBOT_MASS_KG,
                    SwerveHardwareConfig.ROBOT_MOI,
                    new ModuleConfig(
                        SwerveHardwareConfig.WHEEL_DIAMETER_METERS / 2.0,
                        maxSpeed.getAsDouble(),
                        SwerveHardwareConfig.WHEEL_COF,
                        DCMotor.getKrakenX60(1),
                        SwerveHardwareConfig.DRIVE_GEAR_RATIO,
                        SwerveHardwareConfig.DRIVE_CURRENT_LIMIT_AMPS,
                        1),
                    SwerveHardwareConfig.MODULE_TRANSLATIONS
            );
        }

        // this registers all the commands we'll use when executing our
        // selected program
        registerNamedCommands();

        // this is what accepts calculated speeds from the path engine
        // and actually applies them to drive the robot around; if the
        // robot isn't moving at all, start debugging here
        BiConsumer<ChassisSpeeds,DriveFeedforwards> output = (s, f) -> {
            SmartDashboard.putNumber("PathPlanner/SpeedX", Units.metersToFeet(s.vxMetersPerSecond));
            SmartDashboard.putNumber("PathPlanner/SpeedY", Units.metersToFeet(s.vyMetersPerSecond));
            SmartDashboard.putNumber("PathPlanner/SpeedOmega", Units.radiansToDegrees(s.omegaRadiansPerSecond));
            swerve.driveRobotRelative("path-planner", s);
        };

        // this is used to provide feedback to keep the robot on the
        // supplied course; if the robot isn't moving accurately, start
        // debugging here
        PathFollowingController controller = new PPHolonomicDriveController(
                new PIDConstants(translationP.getAsDouble(), 0.0, 0.0),
                new PIDConstants(rotationP.getAsDouble(), 0.0, 0.0));

        Util.log("[auto] configuring AutoBuilder");
        AutoBuilder.configure(
                swerve::getPose,
                swerve::resetPose,
                swerve::getCurrentSpeeds,
                output,
                controller,
                config,

                // PP plans are drawn from the perspective of the blue team;
                // if we are the red team, PP should flip the path for us
                Util::isRedAlliance,

                swerve);
    }

    /**
     * Configures logging for PathPlanner. Publishes poses to SmartDashboard
     * for visualization in PathPlanner and other tools.
     */
    private void configureLogging() {

        // publish current pose for PathPlanner visualization
        PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
            Util.publishPose("PathPlanner-Current", pose);
        });

        // publish target pose for PathPlanner visualization
        PathPlannerLogging.setLogTargetPoseCallback(pose -> {
            Util.publishPose("PathPlanner-Target", pose);
        });
    }

    @Override
    public void periodic() {
        selected = picker.get();
    }

    /**
     * Call this from {@link TimedRobot#autonomousInit()} to create the
     * actual command. We do all the work of loading the program and
     * configuring PathPlanner here, so the field operator doesn't have
     * to wait for it to happen.
     */
    public Command generateCommand() {

        if (selected == null) {
            Util.log("[auto] =================================================");
            Util.log("[auto] NO SELECTED PROGRAM!!!");
            Util.log("[auto] =================================================");
            swerve.resetPose(createEmergencyStartPose());
            return createEmergencyCommand(led);
        }

        else {

            Util.log("[auto] =================================================");
            Util.log("[auto] %s", selected);
            Util.log("[auto] =================================================");

            // this loads the actual program description and links it
            // up with all the other configurations
            command = new PathPlannerAuto(selected);

            // in years past we might mess with the program once it was
            // loaded - for instance, prepending some initialization code
            // or adding some timeouts or other such hackery
            command = decorateAutoCommand(command);
        }

        return command;
    }

//endregion

//region Customization ---------------------------------------------------------

    /*
     * TODO identify all your programs
     *
     * This is the list of all the autonomous programs we have to choose
     * from.
     *
     * There should be one entry for each different program. The value on
     * the left is a "short name" (which gets displayed on the DigitBoard)
     * and the value on the right is the actual filename of the program
     * that you edited using PathPlanner.
     */
    private Map<String,String> getProgramNames() {

        // TODO we can get names from AutoBuilder - can we auto-generate codes?

        // we use a LinkedHashMap so the programs will be shown in the
        // same order as below
        Map<String,String> programs = new LinkedHashMap<>();
        programs.put("NONE", "None");
        programs.put("TEST", "Test");
        return programs;
    }

    /*
     * TODO review all named commands
     *
     * This is where you register all the "named commands" that are used
     * by your different autonomous programs
     */
    private void registerNamedCommands() {

        Util.log("[auto] Registering named commands");

        // unload the hopper
        NamedCommands.registerCommand("Unload",
                ShootingCommands.shootMode(led, ballPath, shooter)
                        .withTimeout(unloadSecs.getAsDouble()));

        // open the hopper
        NamedCommands.registerCommand("OpenHopper",
                ShootingCommands.openHopper(swerve));

        // orient to shoot
        // NOTE: This will wind up in the robot's position not matching up
        // with where PathPlanner "thinks" you are. Subsequent paths may not
        // work the way you expect.
        NamedCommands.registerCommand("OrientToShoot",
                ShootingCommands.orientToShoot(led, swerve));
    }

    /*
     * In previous years we've had to do some "mandatory" startup tasks,
     * like lowering an arm or moving the position of a held gamepiece.
     * We did this by taking the Command created by PathPlanner and adding
     * stuff to the beginning or end.
     */
    private Command decorateAutoCommand(Command autoCommand) {

        String msg = """
                [auto] ================================================
                [auto] done with auto
                [auto] ================================================""";

        return autoCommand
                .andThen(Commands.print(msg));
    }

    /*
     * If the autonomous routine gets buggered up, what do you want the
     * robot to do? Sitting still might be one option, but you might also
     * have some of that "mandatory" stuff to do. Or you might want to try
     * some minimal driving to score points.
     *
     * @see #createEmergencyStartPose()
     */
    private Command createEmergencyCommand(LEDSubsystem led) {
        String msg = """
                [auto] ================================================
                [auto] OH, CRAP, SOMETHING WENT WRONG
                [auto] ================================================""";
        Command log = Commands.print(msg);
        Command blink = led.show(LEDSignal.OH_CRAP_AUTO);
        return log.alongWith(blink);
    }

    /*
     * TODO create an "emergency" start pose
     *
     * The robot gets its starting pose from the autonomous path. If there
     * isn't one, what should you do? You probably at least want to consider
     * guessing the heading of the robot, so the driver has some little bit
     * of control and can e.g. get to an AprilTag and reset position from
     * there using the Limelight.
     */
    private Pose2d createEmergencyStartPose() {

        // example: assume the robot starts facing away from
        // the alliance wall (blue is 0, red is 180)
        return Util.isRedAlliance()
                ? new Pose2d(0.0, 0.0, Rotation2d.k180deg)
                : Pose2d.kZero;
    }
}