package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.Field;

/**
 * Standalone test program that spawns the robot 8 feet from either hub.
 * <p>
 * Use D-pad up/down (arrow keys on keyboard) to cycle through positions.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SpawnTestbot}
 */
public class SpawnTestbot extends TimedRobot {

    static final double DISTANCE_FEET = 8.0;

    final SwerveSubsystem swerve;
    final GameController controller;

    final Pose2d[] positions;
    final String[] labels;
    int currentIndex = 0;

    public SpawnTestbot() {

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        controller = new GameController(0);

        // build the 4 spawn positions
        Translation2d blue = blueHub();
        Translation2d red = redHub();
        positions = new Pose2d[] {
            poseFromHub(blue, 135),
            poseFromHub(blue, -135),
            poseFromHub(red, 45),
            poseFromHub(red, -45),
        };
        labels = new String[] {
            "Blue +135",
            "Blue -135",
            "Red +45",
            "Red -45",
        };

        // spawn at first position
        swerve.resetPose(positions[0]);

        // axis 0 above 0.98 = next position
        new Trigger(() -> controller.getHID().getRawAxis(0) > 0.98)
                .onTrue(Commands.runOnce(() -> {
                    currentIndex = (currentIndex + 1) % positions.length;
                    swerve.resetPose(positions[currentIndex]);
                    System.out.printf(">>> Teleported to %s%n", labels[currentIndex]);
                }));

        // Z key (button A) = teleport to current position
        controller.a().onTrue(Commands.runOnce(() -> {
            swerve.resetPose(positions[currentIndex]);
            System.out.printf(">>> Teleported to %s%n", labels[currentIndex]);
        }));
    }

    /**
     * @return the blue alliance hub center (midpoint of tags 18 and 21)
     */
    static Translation2d blueHub() {
        Pose2d tag1 = Field.getTagPose(18);
        Pose2d tag2 = Field.getTagPose(21);
        return new Translation2d(
                (tag1.getX() + tag2.getX()) / 2.0,
                (tag1.getY() + tag2.getY()) / 2.0);
    }

    /**
     * @return the red alliance hub center (midpoint of tags 2 and 5)
     */
    static Translation2d redHub() {
        Pose2d tag1 = Field.getTagPose(2);
        Pose2d tag2 = Field.getTagPose(5);
        return new Translation2d(
                (tag1.getX() + tag2.getX()) / 2.0,
                (tag1.getY() + tag2.getY()) / 2.0);
    }

    /**
     * Computes a pose that is 8 feet from a hub at the given angle,
     * facing the hub.
     *
     * @param hub the hub center position
     * @param angleDeg angle from hub center in degrees (0 = toward red wall)
     * @return the pose facing the hub
     */
    static Pose2d poseFromHub(Translation2d hub, double angleDeg) {
        double distMeters = Units.feetToMeters(DISTANCE_FEET);
        double angleRad = Math.toRadians(angleDeg);

        Translation2d offset = new Translation2d(
                distMeters * Math.cos(angleRad),
                distMeters * Math.sin(angleRad));
        Translation2d position = hub.plus(offset);

        // face back toward the hub
        Rotation2d heading = offset.getAngle().plus(Rotation2d.k180deg);

        return new Pose2d(position, heading);
    }

    @Override
    public void robotInit() {
        System.out.println(">>> SpawnTestbot starting...");
        System.out.println(">>> D-pad UP / DOWN to cycle positions:");
        for (int i = 0; i < labels.length; i++) {
            Pose2d p = positions[i];
            System.out.printf(">>>   [%d] %s  (%.2f, %.2f)m%n",
                    i, labels[i], p.getX(), p.getY());
        }

        SmartDashboard.putData("SpawnTestbot", builder -> {
            builder.addStringProperty("Position", () -> labels[currentIndex], null);
            builder.addDoubleProperty("X_feet", () -> Units.metersToFeet(swerve.getPose().getX()), null);
            builder.addDoubleProperty("Y_feet", () -> Units.metersToFeet(swerve.getPose().getY()), null);
            builder.addDoubleProperty("Heading_deg", () -> swerve.getHeading().getDegrees(), null);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
