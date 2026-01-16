package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Wraps the {@link TrapezoidProfile} to provide a couple of useful functions:
 * <ul>
 *
 *     <li>Maximum velocity and acceleration are captured from
 *     {@link DoubleSupplier} instances so they can be managed via e.g.
 *     preferences</li>
 *
 *     <li>Remembers start and final states to make sampling the trajectory
 *     simpler</li>
 *
 * </ul>
 */
public class Trapezoid {

    final DoubleSupplier maxVelocity;
    final DoubleSupplier maxAcceleration;
    State startState;
    State finalState;
    TrapezoidProfile profile;

    /**
     * Creates a {@link TrapezoidProfile}
     * @param maxVelocity supplier for maximum velocity in units (required)
     * @param maxAcceleration supplier for maximum acceleration in units per
     *                        second (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public Trapezoid(DoubleSupplier maxVelocity, DoubleSupplier maxAcceleration) {
        this.maxVelocity = Objects.requireNonNull(maxVelocity);
        this.maxAcceleration = Objects.requireNonNull(maxAcceleration);
    }

    /**
     * Calculates a motion profile (start and final velocity are assumed to be 0)
     * @param startPosition start position in units
     * @param finalPosition final position in units
     */
    public void calculate(double startPosition, double finalPosition) {
        calculate(startPosition, 0.0, finalPosition, 0.0);
    }

    /**
     * Calculates a motion profile (final velocity is assumed to be 0)
     * @param startPosition start position in units
     * @param startVelocity start velocity in units per second
     * @param finalPosition final position in units
     */
    public void calculate(double startPosition, double startVelocity, double finalPosition) {
        calculate(startPosition, startVelocity, finalPosition, 0.0);
    }

    /**
     * Calculates a motion profile
     * @param startPosition start position in units
     * @param startVelocity start velocity in units per second
     * @param finalPosition final position in units
     * @param finalVelocity final velocity in units per second
     */
    public void calculate(double startPosition, double startVelocity,
                      double finalPosition, double finalVelocity) {

        startState = new State(startPosition, startVelocity);
        finalState = new State(finalPosition, finalVelocity);
        profile = new TrapezoidProfile(new Constraints(
                maxVelocity.getAsDouble(),
                maxAcceleration.getAsDouble()));

        // we perform this initial calculation to make sure that subsequent
        // calls to e.g. totalTime are accurate
        profile.calculate(0.0, startState, finalState);
    }

    /**
     * @return the total amount of time in seconds the motion will take
     */
    public double totalTime() {
        return profile.totalTime();
    }

    /**
     * @param t a time in seconds
     * @return is the motion finished at that time?
     */
    public boolean isFinishedAt(double t) {
        return t < totalTime();
    }

    /**
     * @param t a time in seconds
     * @return the state of the motion at that time (for t&lt;0 this is the
     * start state, for t&gt;totalTime this is the final state)
     */
    public State sample(double t) {
        if (t < 0) {
            return startState;
        }
        if (t > totalTime()) {
            return finalState;
        }
        return profile.calculate(t, startState, finalState);
    }
}
