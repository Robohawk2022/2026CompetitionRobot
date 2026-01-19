# Session: Add Swerve Cosine Compensation and Angle Motor Optimization

**Date**: 2026-01-19

## Summary

Added cosine compensation to swerve module kinematics and implemented an optimization to skip angle motor movement when the drive motor is stationary. Also fixed several pre-existing compile errors related to the disabled OdometryThread.

## Changes Made

### Files Modified

#### `src/main/java/frc/robot/Config.java`
- Added `Swerve/CosineCompensation?` preference (default: true) - enables/disables cosine compensation
- Added `Swerve/MinSpeedForAngle` preference (default: 0.01 m/s) - threshold below which angle motor doesn't move

#### `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
- Rewrote `setDesiredState()` method to:
  1. Skip angle motor movement if drive speed is below `minSpeedForAngle` threshold (prevents jitter when stationary)
  2. Apply cosine compensation when enabled - scales drive output by `cos(angleError)` to account for wheel misalignment
  3. Removed debug `System.out.println()` statements

#### `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java`
- Added import for `com.ctre.phoenix6.BaseStatusSignal`
- Added `getAllSignals()` default method returning empty array (needed by OdometryThread)

#### `src/main/java/frc/robot/subsystems/swerve/OdometryDiagnostics.java`
- Added overloaded `update(Pose2d, Pose2d, VisionEstimateResult)` method that doesn't require OdometryThread parameter
- Removed unused odometry stats fields (`odometryFrequency`, `lastUpdateCount`, `lastFrequencyCalcTime`)

## Config Changes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Swerve/CosineCompensation?` | true | Enable cosine compensation |
| `Swerve/MinSpeedForAngle` | 0.01 | Min speed (m/s) to command angle motor |

## Technical Details

### Cosine Compensation
When a swerve wheel is pointed away from its target direction, only part of its speed contributes to the desired motion. Cosine compensation scales the drive velocity by `cos(angleError)` so the wheel delivers the correct effective speed:

```java
double speedMps = state.speedMetersPerSecond;
if (cosineCompensation.getAsBoolean()) {
    double angleErrorRad = state.angle.minus(currentAngle).getRadians();
    speedMps *= Math.cos(angleErrorRad);
}
```

### Angle Motor Skip
When the robot is stationary (speed < threshold), continuously commanding the angle motor causes unnecessary wear and "jitter". The fix simply returns early without commanding the turn motor:

```java
if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
    driveMotor.setControl(driveRequest.withVelocity(0));
    desiredState = new SwerveModuleState(0, currentAngle);
    return;
}
```

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

- The OdometryThread class still exists but is not used (intentionally disabled in earlier commit 3c431cf)
- SwerveSubsystem no longer references odometryThread - all odometry runs at 50Hz through standard SwerveDriveOdometry
- Cosine compensation can be disabled via dashboard if it causes issues during testing
