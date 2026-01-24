# Session: Orbit Command Rotation Priority

**Date**: 2026-01-23

## Summary

Implemented rotation priority for the orbit command to prevent translation from starving heading control when rotation demands are high.

## Problem

When orbiting at high speed with large heading error:
1. Robot was slow to turn to face the target
2. `desaturateWheelSpeeds` scales all demands proportionally, causing rotation to be limited

## Solution

Added pre-desaturation step that scales translation down when rotation needs significant authority.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` - Added `rotationPriority` parameter to Orbit interface
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added rotation priority logic before creating ChassisSpeeds

## Config Changes

- Added `Orbit/RotationPriority` with default value 0.5

## How It Works

The new logic in `orbitCommand()`:

```java
// prioritize rotation over translation when heading error is large
double omegaFraction = Math.abs(omega) / maxRotationRps;
double rotPriority = rotationPriority.getAsDouble();
if (omegaFraction > rotPriority) {
    double translationScale = 1.0 - (omegaFraction - rotPriority);
    translationScale = Math.max(translationScale, 0.3); // never below 30%
    vxField *= translationScale;
    vyField *= translationScale;
}
```

With default `rotationPriority = 0.5`:
- If rotation uses 50% or less of max omega: no scaling
- If rotation uses 70% of max omega: translation scaled to 80%
- If rotation uses 90% of max omega: translation scaled to 60%
- Translation never goes below 30% regardless of rotation demand

## Tuning Guide

- **Increase `Orbit/RotationPriority`** (e.g., 0.6-0.7): More aggressive rotation priority, faster heading correction but slower orbit speed
- **Decrease `Orbit/RotationPriority`** (e.g., 0.3-0.4): Less rotation priority, maintains higher orbit speed but slower heading correction
- **Alternative**: Increase `Orbit/Heading/kP` from 6.0 to 10.0-15.0 for stronger rotation demand without the priority system

## Testing Done

- [x] `./gradlew build` - passed

## Verification Steps

1. Run simulation: `./gradlew simulateJava`
2. Enable orbit mode (left bumper)
3. Drive tangentially at high speed
4. Check `Swerve/Orbit/HeadingErrorDeg` stays small (< 10°) while moving

## Notes

The 30% minimum translation scale ensures the robot can still move even when fully correcting heading. This value could be made configurable if needed.
