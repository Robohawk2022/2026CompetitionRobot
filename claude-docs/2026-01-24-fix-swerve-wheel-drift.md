# Session: Fix Swerve Wheel Drift Bug

**Date**: 2026-01-24

## Summary

Fixed a bug in `SwerveModule.setDesiredState()` where the turn motor was not commanded when drive speed dropped below the `minSpeedForAngle` threshold, causing wheels to drift and produce diagonal movement on subsequent forward commands.

## Problem Description

When running the swerve test bot and only going forward/backward:
- Some wheels would end up at unexpected angles
- The next forward command would cause the robot to move diagonally

## Root Cause

In `SwerveModule.setDesiredState()` (lines 219-223), when drive speed was below the threshold:
1. The drive motor was commanded to 0 velocity (correct)
2. BUT the turn motor was **not commanded at all** - just held its previous setpoint
3. If wheels drifted (vibration, motor settling, external forces), they weren't corrected
4. Different modules dropping below the threshold at different times could end up with inconsistent angles

This also broke `setX()` which tried to set specific angles with speed=0, triggering the early return and never actually commanding the X-pattern.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Fixed the low-speed handling to still command the turn motor to the desired angle

**Before:**
```java
if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
    driveMotor.setControl(driveRequest.withVelocity(0));
    desiredState = new SwerveModuleState(0, currentAngle);
    return;  // Turn motor NOT commanded
}
```

**After:**
```java
if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
    driveMotor.setControl(driveRequest.withVelocity(0));
    // still command turn motor to the desired angle
    double positionRotations = state.angle.getRotations();
    turnMotor.setControl(turnRequest.withPosition(positionRotations));
    desiredState = new SwerveModuleState(0, state.angle);
    return;
}
```

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not yet tested
- [ ] Real hardware - not yet tested

## Additional Notes

### Potential Calibration Issue (Not Fixed)

I also noticed that:
1. `configureCANcoder()` is commented out in `SwerveModule` constructor (line 88)
2. All angular offsets in `SwerveHardwareConfig.java` are set to 0

If the CANcoder magnets aren't physically aligned with "forward" when they read 0, the wheels may not point correctly. This would require physical calibration:
1. Point all wheels forward manually
2. Read CANcoder absolute positions
3. Update the angular offset constants in `SwerveHardwareConfig.java`
4. Uncomment `configureCANcoder()` or configure offsets via CTRE Tuner

This is a separate issue from the software bug fixed above.

## Notes for Next Session

- Test the fix in simulation and on real hardware
- If diagonal drift persists, investigate CANcoder calibration
- Monitor dashboard values: `SwerveModules/*/AngleCurrent` and `AngleDesired` to verify turn motors are holding position correctly
