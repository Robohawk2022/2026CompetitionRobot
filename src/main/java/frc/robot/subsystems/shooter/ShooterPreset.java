package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import static frc.robot.Config.Shooter.speed1;
import static frc.robot.Config.Shooter.speed2;
import static frc.robot.Config.Shooter.speed3;

/**
 * Represents a preset linear speed for the arm, with a value derived from
 * configuration
 */
public enum ShooterPreset {

    S1,
    S2,
    S3;

    /**
     * @return the linear speed corresponding to this preset in feet per
     * second
     */
    public double feetPerSecond() {
        DoubleSupplier speed = switch (this) {
            case S1 -> speed1;
            case S2 -> speed2;
            case S3 -> speed3;
        };
        return speed.getAsDouble();
    }
}