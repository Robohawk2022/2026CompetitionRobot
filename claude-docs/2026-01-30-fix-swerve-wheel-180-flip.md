# Session: Fix Swerve Wheel 180° Flip After Stopping

**Date**: 2026-01-30

## Summary

Fixed a bug where swerve wheel target headings would flip 180 degrees when the driver released the joystick, instead of staying at their last commanded angle.

## Problem Description

When the driver released the controller after moving, the swerve wheel target headings could flip 180 degrees instead of staying at their current angle.

## Root Cause

In `SwerveModule.setDesiredState()`, when drive speed was below the `minSpeedForAngle` threshold, the code was using `state.angle` from kinematics. The problem is that when `ChassisSpeeds` is zero (no joystick input), `KINEMATICS.toSwerveModuleStates(speeds)` produces arbitrary/undefined angles because there's no direction of travel.

Additionally, the `state.optimize(currentAngle)` call that prevents >90° rotations happens **after** the low-speed check, so it never runs when speed is zero.

## Changes Made

### Files Modified

**`src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`**

Changed the low-speed handling to preserve the **previous** desired angle instead of using the arbitrary angle from kinematics.

Before:
```java
if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
    driveMotor.setControl(driveRequest.withVelocity(0));
    double positionRotations = state.angle.getRotations();  // arbitrary kinematics angle
    turnMotor.setControl(turnRequest.withPosition(positionRotations));
    desiredState = new SwerveModuleState(0, state.angle);
    return;
}
```

After:
```java
if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
    driveMotor.setControl(driveRequest.withVelocity(0));
    // preserve the previous desired angle - don't flip to arbitrary kinematics angle
    double positionRotations = desiredState.angle.getRotations();  // last commanded angle
    turnMotor.setControl(turnRequest.withPosition(positionRotations));
    desiredState = new SwerveModuleState(0, desiredState.angle);
    return;
}
```

**`src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java`**

Updated `setX()` to use `setTurnAngle()` directly, since `setDesiredState()` now preserves angles at zero speed.

Before:
```java
public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // ... etc
}
```

After:
```java
public void setX() {
    // stop all drive motors (setDesiredState with 0 speed preserves current angle)
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeft.getState().angle));
    frontRight.setDesiredState(new SwerveModuleState(0, frontRight.getState().angle));
    backLeft.setDesiredState(new SwerveModuleState(0, backLeft.getState().angle));
    backRight.setDesiredState(new SwerveModuleState(0, backRight.getState().angle));

    // explicitly set X-pattern angles using setTurnAngle
    frontLeft.setTurnAngle(Rotation2d.fromDegrees(45));
    frontRight.setTurnAngle(Rotation2d.fromDegrees(-45));
    backLeft.setTurnAngle(Rotation2d.fromDegrees(-45));
    backRight.setTurnAngle(Rotation2d.fromDegrees(45));
}
```

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not yet tested
- [ ] Real hardware - not yet tested

## Verification Steps

To test the fix:
1. Run `./gradlew simulateJava -Probot=SwerveTestbot`
2. Drive forward, release stick → wheels should stay pointed forward
3. Drive diagonally, release stick → wheels should maintain diagonal angle
4. Press lock wheels button → wheels should form X pattern

## Notes for Next Session

- Test in simulation and on real hardware
- Monitor dashboard values: `SwerveModules/*/AngleCurrent` and `AngleDesired` to verify wheels hold position after release
- If wheels still flip, check that `desiredState.angle` is properly initialized at module startup
