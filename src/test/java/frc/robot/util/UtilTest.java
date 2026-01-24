package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link Util} class.
 */
class UtilTest {

    // tolerance for floating point comparisons
    static final double EPSILON = 1e-9;

    //region clampVolts tests ------------------------------------------------------

    @Test
    void clampVolts_withinRange_returnsUnchanged() {
        assertEquals(5.0, Util.clampVolts(5.0), EPSILON);
        assertEquals(-5.0, Util.clampVolts(-5.0), EPSILON);
        assertEquals(0.0, Util.clampVolts(0.0), EPSILON);
    }

    @Test
    void clampVolts_atBoundary_returnsUnchanged() {
        assertEquals(12.0, Util.clampVolts(12.0), EPSILON);
        assertEquals(-12.0, Util.clampVolts(-12.0), EPSILON);
    }

    @Test
    void clampVolts_exceedsMax_clampsToMax() {
        assertEquals(12.0, Util.clampVolts(15.0), EPSILON);
        assertEquals(12.0, Util.clampVolts(100.0), EPSILON);
        assertEquals(12.0, Util.clampVolts(Double.MAX_VALUE), EPSILON);
    }

    @Test
    void clampVolts_exceedsMin_clampsToMin() {
        assertEquals(-12.0, Util.clampVolts(-15.0), EPSILON);
        assertEquals(-12.0, Util.clampVolts(-100.0), EPSILON);
        assertEquals(-12.0, Util.clampVolts(-Double.MAX_VALUE), EPSILON);
    }

    //endregion

    //region applyClamp tests ------------------------------------------------------

    @Test
    void applyClamp_withinSymmetricLimit_returnsUnchanged() {
        assertEquals(5.0, Util.applyClamp(5.0, () -> 10.0), EPSILON);
        assertEquals(-5.0, Util.applyClamp(-5.0, () -> 10.0), EPSILON);
    }

    @Test
    void applyClamp_exceedsSymmetricLimit_clamps() {
        assertEquals(10.0, Util.applyClamp(15.0, () -> 10.0), EPSILON);
        assertEquals(-10.0, Util.applyClamp(-15.0, () -> 10.0), EPSILON);
    }

    @Test
    void applyClamp_asymmetricBounds_clampsCorrectly() {
        assertEquals(5.0, Util.applyClamp(10.0, () -> 0.0, () -> 5.0), EPSILON);
        assertEquals(0.0, Util.applyClamp(-5.0, () -> 0.0, () -> 5.0), EPSILON);
        assertEquals(2.5, Util.applyClamp(2.5, () -> 0.0, () -> 5.0), EPSILON);
    }

    //endregion

    //region degreeModulus tests ---------------------------------------------------

    @Test
    void degreeModulus_withinRange_returnsUnchanged() {
        assertEquals(0.0, Util.degreeModulus(0.0), EPSILON);
        assertEquals(90.0, Util.degreeModulus(90.0), EPSILON);
        assertEquals(-90.0, Util.degreeModulus(-90.0), EPSILON);
        assertEquals(179.0, Util.degreeModulus(179.0), EPSILON);
        assertEquals(-179.0, Util.degreeModulus(-179.0), EPSILON);
    }

    @Test
    void degreeModulus_exceeds180_wrapsCorrectly() {
        assertEquals(-179.0, Util.degreeModulus(181.0), EPSILON);
        assertEquals(-90.0, Util.degreeModulus(270.0), EPSILON);
        assertEquals(0.0, Util.degreeModulus(360.0), EPSILON);
        assertEquals(90.0, Util.degreeModulus(450.0), EPSILON);
    }

    @Test
    void degreeModulus_belowNeg180_wrapsCorrectly() {
        assertEquals(179.0, Util.degreeModulus(-181.0), EPSILON);
        assertEquals(90.0, Util.degreeModulus(-270.0), EPSILON);
        assertEquals(0.0, Util.degreeModulus(-360.0), EPSILON);
        assertEquals(-90.0, Util.degreeModulus(-450.0), EPSILON);
    }

    @Test
    void degreeModulus_multipleRotations_wrapsCorrectly() {
        assertEquals(45.0, Util.degreeModulus(405.0), EPSILON);
        assertEquals(45.0, Util.degreeModulus(765.0), EPSILON);
        assertEquals(-45.0, Util.degreeModulus(-405.0), EPSILON);
    }

    //endregion

    //region nearZero tests --------------------------------------------------------

    @Test
    void nearZero_withinTolerance_returnsTrue() {
        assertTrue(Util.nearZero(0.0, () -> 0.1));
        assertTrue(Util.nearZero(0.05, () -> 0.1));
        assertTrue(Util.nearZero(-0.05, () -> 0.1));
    }

    @Test
    void nearZero_outsideTolerance_returnsFalse() {
        assertFalse(Util.nearZero(0.2, () -> 0.1));
        assertFalse(Util.nearZero(-0.2, () -> 0.1));
    }

    @Test
    void nearZero_atTolerance_returnsFalse() {
        // at exactly the tolerance, abs(value) is NOT < tolerance
        assertFalse(Util.nearZero(0.1, () -> 0.1));
        assertFalse(Util.nearZero(-0.1, () -> 0.1));
    }

    //endregion

    //region linearSpeedToMotorSpeed tests -----------------------------------------

    @Test
    void linearSpeedToMotorSpeed_basicConversion() {
        // wheel diameter = 1/PI means circumference = 1
        // with gear ratio 1:1, 1 unit/s linear = 1 rev/s motor
        double result = Util.linearSpeedToMotorSpeed(1.0, 1.0, 1.0 / Math.PI);
        assertEquals(1.0, result, EPSILON);
    }

    @Test
    void linearSpeedToMotorSpeed_withGearRatio() {
        // 10:1 gear ratio means motor spins 10x faster
        double wheelDiameter = 1.0 / Math.PI;  // circumference = 1
        double result = Util.linearSpeedToMotorSpeed(1.0, 10.0, wheelDiameter);
        assertEquals(10.0, result, EPSILON);
    }

    @Test
    void linearSpeedToMotorSpeed_zeroSpeed_returnsZero() {
        assertEquals(0.0, Util.linearSpeedToMotorSpeed(0.0, 10.0, 0.1), EPSILON);
    }

    //endregion

    //region motorSpeedToLinearSpeed tests -----------------------------------------

    @Test
    void motorSpeedToLinearSpeed_basicConversion() {
        // wheel diameter = 1/PI means circumference = 1
        // with gear ratio 1:1, 1 rev/s motor = 1 unit/s linear
        double result = Util.motorSpeedToLinearSpeed(1.0, 1.0, 1.0 / Math.PI);
        assertEquals(1.0, result, EPSILON);
    }

    @Test
    void motorSpeedToLinearSpeed_withGearRatio() {
        // 10:1 gear ratio means wheel spins 1/10th motor speed
        double wheelDiameter = 1.0 / Math.PI;  // circumference = 1
        double result = Util.motorSpeedToLinearSpeed(10.0, 10.0, wheelDiameter);
        assertEquals(1.0, result, EPSILON);
    }

    @Test
    void motorSpeedToLinearSpeed_roundTrip() {
        // converting linear -> motor -> linear should give original value
        double original = 5.5;
        double gearRatio = 8.45;
        double wheelDiameter = 0.1;

        double motorSpeed = Util.linearSpeedToMotorSpeed(original, gearRatio, wheelDiameter);
        double recovered = Util.motorSpeedToLinearSpeed(motorSpeed, gearRatio, wheelDiameter);

        assertEquals(original, recovered, EPSILON);
    }

    //endregion

    //region distance calculations -------------------------------------------------

    @Test
    void metersBetween_samePose_returnsZero() {
        Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        assertEquals(0.0, Util.metersBetween(pose, pose), EPSILON);
    }

    @Test
    void metersBetween_horizontalDistance() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0));
        assertEquals(3.0, Util.metersBetween(start, end), EPSILON);
    }

    @Test
    void metersBetween_verticalDistance() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(0.0, 4.0, Rotation2d.fromDegrees(0));
        assertEquals(4.0, Util.metersBetween(start, end), EPSILON);
    }

    @Test
    void metersBetween_diagonalDistance() {
        // 3-4-5 triangle
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
        assertEquals(5.0, Util.metersBetween(start, end), EPSILON);
    }

    @Test
    void feetBetween_convertsProperly() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0));  // 1 meter apart

        double feet = Util.feetBetween(start, end);
        assertEquals(Units.metersToFeet(1.0), feet, EPSILON);
    }

    //endregion

    //region rotation calculations -------------------------------------------------

    @Test
    void degreesBetween_samePose_returnsZero() {
        Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        assertEquals(0.0, Util.degreesBetween(pose, pose), EPSILON);
    }

    @Test
    void degreesBetween_positiveRotation() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90));
        assertEquals(90.0, Util.degreesBetween(start, end), EPSILON);
    }

    @Test
    void degreesBetween_negativeRotation() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90));
        Pose2d end = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        assertEquals(-90.0, Util.degreesBetween(start, end), EPSILON);
    }

    @Test
    void radiansBetween_convertsCorrectly() {
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90));

        double radians = Util.radiansBetween(start, end);
        assertEquals(Math.PI / 2.0, radians, EPSILON);
    }

    //endregion

    //region clampFeetAndDegrees tests ---------------------------------------------

    @Test
    void clampFeetAndDegrees_withinLimits_unchanged() {
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 1.0, 0.5);
        Util.clampFeetAndDegrees(speeds, () -> 10.0, () -> 90.0);

        // speeds should be unchanged (1,1 magnitude ~= 1.41 m/s = ~4.6 ft/s < 10)
        assertEquals(1.0, speeds.vxMetersPerSecond, EPSILON);
        assertEquals(1.0, speeds.vyMetersPerSecond, EPSILON);
    }

    @Test
    void clampFeetAndDegrees_exceedsTranslation_scales() {
        // create speeds that exceed the limit
        double speedMeters = Units.feetToMeters(10.0);  // 10 ft/s in one axis
        ChassisSpeeds speeds = new ChassisSpeeds(speedMeters, 0.0, 0.0);

        Util.clampFeetAndDegrees(speeds, () -> 5.0, () -> 180.0);  // limit to 5 ft/s

        // should be scaled down to 5 ft/s
        double resultFps = Units.metersToFeet(speeds.vxMetersPerSecond);
        assertEquals(5.0, resultFps, EPSILON);
    }

    @Test
    void clampFeetAndDegrees_exceedsRotation_clamps() {
        double rotationRads = Units.degreesToRadians(200.0);  // 200 deg/s
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationRads);

        Util.clampFeetAndDegrees(speeds, () -> 10.0, () -> 90.0);  // limit to 90 deg/s

        double resultDps = Units.radiansToDegrees(speeds.omegaRadiansPerSecond);
        assertEquals(90.0, resultDps, EPSILON);
    }

    //endregion

    //region constants tests -------------------------------------------------------

    @Test
    void constants_haveExpectedValues() {
        assertEquals(12.0, Util.MAX_VOLTS, EPSILON);
        assertEquals(0.02, Util.DT, EPSILON);
    }

    @Test
    void nanRotation_hasNaN() {
        assertTrue(Double.isNaN(Util.NAN_ROTATION.getRadians()));
    }

    @Test
    void nanPose_hasNaNValues() {
        assertTrue(Double.isNaN(Util.NAN_POSE.getX()));
        assertTrue(Double.isNaN(Util.NAN_POSE.getY()));
        assertTrue(Double.isNaN(Util.NAN_POSE.getRotation().getRadians()));
    }

    @Test
    void zeroSpeed_isZero() {
        assertEquals(0.0, Util.ZERO_SPEED.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, Util.ZERO_SPEED.vyMetersPerSecond, EPSILON);
        assertEquals(0.0, Util.ZERO_SPEED.omegaRadiansPerSecond, EPSILON);
    }

    //endregion

    //region incrementPose tests ---------------------------------------------------

    @Test
    void incrementPose_zeroSpeed_unchanged() {
        Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        Pose2d result = Util.incrementPose(pose, speeds);

        assertEquals(pose.getX(), result.getX(), EPSILON);
        assertEquals(pose.getY(), result.getY(), EPSILON);
        assertEquals(pose.getRotation().getDegrees(), result.getRotation().getDegrees(), EPSILON);
    }

    @Test
    void incrementPose_withTranslation_movesCorrectly() {
        Pose2d pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        // 1 m/s for DT seconds = 0.02 meters
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 2.0, 0.0);

        Pose2d result = Util.incrementPose(pose, speeds);

        assertEquals(1.0 * Util.DT, result.getX(), EPSILON);
        assertEquals(2.0 * Util.DT, result.getY(), EPSILON);
    }

    @Test
    void incrementPose_withRotation_rotatesCorrectly() {
        Pose2d pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        // 1 rad/s for DT seconds
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 1.0);

        Pose2d result = Util.incrementPose(pose, speeds);

        double expectedDegrees = Units.radiansToDegrees(1.0 * Util.DT);
        assertEquals(expectedDegrees, result.getRotation().getDegrees(), EPSILON);
    }

    //endregion
}
