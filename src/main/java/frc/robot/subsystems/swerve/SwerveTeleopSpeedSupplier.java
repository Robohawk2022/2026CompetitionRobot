package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameController;

import static frc.robot.Config.Swerve.*;

/**
 * Encapsulates teleop joystick input handling for swerve drive.
 * <p>
 * Handles:
 * <ul>
 *   <li>Left stick Y for forward/backward (inverted)</li>
 *   <li>Left stick X for strafe (inverted)</li>
 *   <li>Right stick X for rotation (inverted)</li>
 *   <li>Left trigger for sniper mode</li>
 *   <li>Right trigger for turbo mode</li>
 * </ul>
 * <p>
 * Applies deadband from {@code Config.Swerve.deadzone}.
 */
public class SwerveTeleopSpeedSupplier {

    private final GameController controller;

    /**
     * Creates a {@link SwerveTeleopSpeedSupplier}.
     *
     * @param controller the game controller for driver input (required)
     */
    public SwerveTeleopSpeedSupplier(GameController controller) {
        this.controller = Objects.requireNonNull(controller);
    }

    /**
     * @return supplier for forward/backward speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier xSupplier() {
        return () -> -MathUtil.applyDeadband(controller.getLeftY(), deadzone.getAsDouble());
    }

    /**
     * @return supplier for left/right strafe speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier ySupplier() {
        return () -> -MathUtil.applyDeadband(controller.getLeftX(), deadzone.getAsDouble());
    }

    /**
     * @return supplier for rotation speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier rotSupplier() {
        return () -> -MathUtil.applyDeadband(controller.getRightX(), deadzone.getAsDouble());
    }

    /**
     * @return supplier for sniper mode trigger axis (0 to 1)
     */
    public BooleanSupplier sniperTrigger() {
        return controller.leftTriggerSupplier();
    }

    /**
     * @return supplier for turbo mode trigger axis (0 to 1)
     */
    public BooleanSupplier turboTrigger() {
        return controller.rightTriggerSupplier();
    }

    /**
     * Creates a teleop drive command with turbo and sniper modes.
     *
     * @param swerve the swerve subsystem to control
     * @return the drive command (field-relative)
     */
    public Command driveCommand(SwerveSubsystem swerve) {
        return driveCommand(swerve, true);
    }

    /**
     * Creates a teleop drive command with turbo and sniper modes.
     *
     * @param swerve        the swerve subsystem to control
     * @param fieldRelative whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(SwerveSubsystem swerve, boolean fieldRelative) {
        return swerve.driveCommand(
                xSupplier(),
                ySupplier(),
                rotSupplier(),
                sniperTrigger(),
                turboTrigger(),
                fieldRelative);
    }

    /**
     * Creates an orbit drive command using controller inputs.
     * <p>
     * Maps controls as follows:
     * <ul>
     *   <li>Left stick Y: Radial movement (toward/away from target)</li>
     *   <li>Left stick X: Tangential movement (orbit around target)</li>
     *   <li>Right stick X: Fine heading adjustment</li>
     *   <li>Triggers: Turbo/sniper speed modifiers</li>
     * </ul>
     *
     * @param swerve the swerve subsystem to control
     * @return the orbit command
     */
    public Command orbitCommand(SwerveSubsystem swerve) {
        return swerve.orbitCommand(
                xSupplier(),     // radial (forward/back = toward/away from target)
                ySupplier(),     // tangent (left/right = orbit around target)
                rotSupplier(),   // rotation trim
                sniperTrigger(),
                turboTrigger());
    }

    /**
     * Creates a face-target drive command using controller inputs.
     * <p>
     * Normal field-relative driving but the robot automatically rotates to face the target (Tower).
     * <ul>
     *   <li>Left stick: Normal translation (field-relative)</li>
     *   <li>Right stick X: Fine heading adjustment</li>
     *   <li>Triggers: Turbo/sniper speed modifiers</li>
     * </ul>
     *
     * @param swerve the swerve subsystem to control
     * @return the face-target drive command
     */
    public Command faceTargetDriveCommand(SwerveSubsystem swerve) {
        return swerve.faceTargetDriveCommand(
                xSupplier(),
                ySupplier(),
                rotSupplier(),
                sniperTrigger(),
                turboTrigger());
    }

    //region Button triggers (delegated to GameController) ---------------------

    /** @return trigger for A button */
    public Trigger a() {
        return controller.a();
    }

    /** @return trigger for B button */
    public Trigger b() {
        return controller.b();
    }

    /** @return trigger for X button */
    public Trigger x() {
        return controller.x();
    }

    /** @return trigger for Y button */
    public Trigger y() {
        return controller.y();
    }

    /** @return trigger for left bumper */
    public Trigger leftBumper() {
        return controller.leftBumper();
    }

    /** @return trigger for right bumper */
    public Trigger rightBumper() {
        return controller.rightBumper();
    }

    /** @return trigger for back/select button */
    public Trigger back() {
        return controller.back();
    }

    /** @return trigger for start button */
    public Trigger start() {
        return controller.start();
    }

    /** @return trigger for left stick click */
    public Trigger leftStick() {
        return controller.leftStick();
    }

    /** @return trigger for right stick click */
    public Trigger rightStick() {
        return controller.rightStick();
    }

    //endregion
}
