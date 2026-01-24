package frc.robot.util;

import org.junit.jupiter.api.Test;

import java.util.function.DoubleSupplier;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link PDController} class.
 */
class PDControllerTest {

    static final double EPSILON = 1e-9;

    //region constructor tests -----------------------------------------------------

    @Test
    void constructor_setsGainsFromSuppliers() {
        PDController controller = new PDController(() -> 1.5, () -> 0.5, () -> 0.1);

        assertEquals(1.5, controller.getP(), EPSILON);
        assertEquals(0.5, controller.getD(), EPSILON);
        assertEquals(0.0, controller.getI(), EPSILON);  // I should always be 0
    }

    @Test
    void constructor_setsTolerance() {
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 0.25);

        assertEquals(0.25, controller.getErrorTolerance(), EPSILON);
    }

    @Test
    void constructor_nullTolerance_doesNotThrow() {
        assertDoesNotThrow(() -> new PDController(() -> 1.0, () -> 0.0, null));
    }

    //endregion

    //region reset tests -----------------------------------------------------------

    @Test
    void reset_updatesGainsFromSuppliers() {
        // use mutable values to change supplier output
        double[] p = {1.0};
        double[] d = {0.1};
        double[] tol = {0.5};

        PDController controller = new PDController(
            () -> p[0],
            () -> d[0],
            () -> tol[0]
        );

        // change the values
        p[0] = 2.0;
        d[0] = 0.2;
        tol[0] = 1.0;

        // values should still be old until reset
        assertEquals(1.0, controller.getP(), EPSILON);

        // now reset
        controller.reset();

        // values should be updated
        assertEquals(2.0, controller.getP(), EPSILON);
        assertEquals(0.2, controller.getD(), EPSILON);
        assertEquals(1.0, controller.getErrorTolerance(), EPSILON);
    }

    @Test
    void reset_clearsDerivativeState() {
        PDController controller = new PDController(() -> 0.0, () -> 1.0, () -> 0.1);

        // calculate a few times to build up derivative history
        controller.calculate(0.0, 10.0);
        controller.calculate(5.0, 10.0);

        // reset should clear state
        controller.reset();

        // first calculate after reset should have no derivative contribution
        // since d-term needs previous measurement
        double output = controller.calculate(0.0, 0.0);
        assertEquals(0.0, output, EPSILON);
    }

    //endregion

    //region withinTolerance tests -------------------------------------------------

    @Test
    void withinTolerance_atSetpoint_returnsTrue() {
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 0.1);

        assertTrue(controller.withinTolerance(10.0, 10.0));
    }

    @Test
    void withinTolerance_withinTolerance_returnsTrue() {
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 0.1);

        assertTrue(controller.withinTolerance(10.05, 10.0));
        assertTrue(controller.withinTolerance(9.95, 10.0));
    }

    @Test
    void withinTolerance_outsideTolerance_returnsFalse() {
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 0.1);

        assertFalse(controller.withinTolerance(10.2, 10.0));
        assertFalse(controller.withinTolerance(9.8, 10.0));
    }

    @Test
    void withinTolerance_atToleranceBoundary_returnsTrue() {
        // MathUtil.isNear uses <= for comparison
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 0.1);

        assertTrue(controller.withinTolerance(10.1, 10.0));
        assertTrue(controller.withinTolerance(9.9, 10.0));
    }

    //endregion

    //region calculate tests -------------------------------------------------------

    @Test
    void calculate_proportionalOnly_outputsError() {
        PDController controller = new PDController(() -> 2.0, () -> 0.0, () -> 0.1);
        controller.reset();

        // with P=2 and error of 5, output should be 10
        double output = controller.calculate(0.0, 5.0);

        assertEquals(10.0, output, EPSILON);
    }

    @Test
    void calculate_atSetpoint_outputsZero() {
        PDController controller = new PDController(() -> 2.0, () -> 0.0, () -> 0.1);
        controller.reset();

        double output = controller.calculate(10.0, 10.0);

        assertEquals(0.0, output, EPSILON);
    }

    @Test
    void calculate_negativeError_outputsNegative() {
        PDController controller = new PDController(() -> 2.0, () -> 0.0, () -> 0.1);
        controller.reset();

        // measurement > setpoint means we need negative output
        double output = controller.calculate(15.0, 10.0);

        assertEquals(-10.0, output, EPSILON);
    }

    @Test
    void calculate_withDerivative_dampensResponse() {
        PDController controller = new PDController(() -> 1.0, () -> 0.5, () -> 0.1);
        controller.reset();

        // first call establishes baseline
        controller.calculate(0.0, 10.0);

        // second call with measurement moving toward setpoint
        // derivative should reduce output (damping)
        double output = controller.calculate(2.0, 10.0);

        // P-term alone would be 8.0 (error = 10 - 2 = 8)
        // D-term is negative since error is decreasing
        assertTrue(output < 8.0, "D-term should dampen response");
    }

    //endregion

    //region I term is always zero -------------------------------------------------

    @Test
    void iGain_alwaysZero() {
        PDController controller = new PDController(() -> 1.0, () -> 1.0, () -> 0.1);

        assertEquals(0.0, controller.getI(), EPSILON);

        // even after reset
        controller.reset();
        assertEquals(0.0, controller.getI(), EPSILON);
    }

    //endregion

    //region continuous input mode -------------------------------------------------

    @Test
    void setContinuousInput_worksForAngles() {
        PDController controller = new PDController(() -> 1.0, () -> 0.0, () -> 1.0);
        controller.enableContinuousInput(-180.0, 180.0);
        controller.reset();

        // going from 170 to -170 should be a small positive move (20 degrees)
        // not a large negative move (-340 degrees)
        double output = controller.calculate(170.0, -170.0);

        // error should be treated as 20, not -340
        // with P=1.0, output should be 20
        assertEquals(20.0, output, EPSILON);
    }

    //endregion
}
