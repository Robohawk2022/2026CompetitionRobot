package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

/**
 * Adds the following useful functionality to a normal {@link PIDController}:
 * <ul>
 *
 *     <li>It doesn't use I - we generally stay away from it because of
 *     <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/common-control-issues.html#integral-term-windup">integral
 *     windup</a></li>
 *
 *     <li>Parameters are received from {@link DoubleSupplier} instances,
 *     so they can be managed via {@link edu.wpi.first.wpilibj.Preferences}.
 *     Updated values are read on {@link #reset()}.
 *     </li>
 *
 *     <li>Adds a helper method to check a measurement and setpoint without
 *     actually changing the existing setpoint.</li>
 *
 * </ul>
 */
public class PDController extends PIDController {

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier tolerance;

    /**
     * Creates a {@link PDController} with no max feedback
     * @param p supplier for the p parameter
     * @param d supplier for the d parameter
     * @param tolerance supplier for the tolerance parameter
     */
    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier tolerance) {
        super(p.getAsDouble(), 0.0, d.getAsDouble());
        this.p = p;
        this.d = d;
        this.tolerance = tolerance;
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
    }

    /**
     * Resets all parameters from their associated {@link DoubleSupplier} and
     * accumulated error
     */
    @Override
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
        super.reset();
    }

    /**
     * Helper to allow comparing values using the tolerance configured for
     * this controller
     *
     * @param measurement a measured value
     * @param setpoint a target value
     * @return is the measurement "close enough" to the setpoint according
     * to the currently configured tolerance?
     */
    public boolean withinTolerance(double measurement, double setpoint) {
        return MathUtil.isNear(setpoint, measurement, getErrorTolerance());
    }
}
