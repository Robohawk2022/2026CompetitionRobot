# Session: Orbit/Face-Target Rotation Priority

**Date**: 2026-01-23

## Summary

Improved heading control for orbit and face-target commands to always face the target while moving. Robot now prioritizes rotation over translation when heading error is large.

## Problem

Robot was slow to turn and face the target when heading error was large, especially while moving at high speed. Translation was consuming wheel capacity, starving rotation.

## Solution

1. Increased heading kP from 6.0 to 36.0 for aggressive rotation (max speed at 10° error)
2. Removed gradual PDController approach - now uses direct proportional with saturation
3. Created custom `drivePreserveRotation()` method that prioritizes rotation over translation
4. Applied changes to BOTH `orbitCommand` and `faceTargetDriveCommand`

### Key Innovation: drivePreserveRotation()

The standard `desaturateWheelSpeeds` scales rotation AND translation equally when wheels exceed capacity. This caused heading to drift during fast movement.

The new `drivePreserveRotation()` method:
1. Calculates wheel speed needed for rotation alone
2. Subtracts from total capacity to get remaining capacity for translation
3. Scales translation to fit remaining capacity
4. Rotation is NEVER scaled down

This ensures the robot always tracks the target heading, even if it has to slow down translation to do so.

## Changes Made

### Files Modified

**`src/main/java/frc/robot/Config.java`**
- `Orbit/Heading/kP`: 6.0 → 36.0 (max rotation at 10° error)
- Added `Orbit/RotationPriority`: 0.5 (preserves rotation authority)
- `Swerve/UseXboxMapping?`: false → true (for simulation)

**`src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`**
- Updated `orbitCommand`: aggressive heading control + rotation priority
- Updated `faceTargetDriveCommand`: aggressive heading control + rotation priority
- Removed PDController from both commands (direct proportional with saturation)
- Both commands now update orbit telemetry for dashboard monitoring

**`src/main/java/frc/robot/RobotContainer.java`**
- Consolidated to single SwerveTeleopSpeedSupplier instance

## Config Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Orbit/Heading/kP` | 72.0 | Hits max rotation at 5° error (very aggressive) |
| `Orbit/RotationPriority` | 0.5 | Start scaling translation at 50% rotation |
| `Swerve/UseXboxMapping?` | false | 8BitDo controller mapping (real robot) |

## How It Works

### Aggressive Heading Control
```java
// Direct proportional with saturation to max rotation speed
double maxOmegaDps = Math.toDegrees(maxRotationRps);
double omegaDps = headingKP.getAsDouble() * headingError;
omegaDps = MathUtil.clamp(omegaDps, -maxOmegaDps, maxOmegaDps);
```

### Rotation Priority
```java
// Scale down translation when rotation needs authority
double omegaFraction = Math.abs(omega) / maxRotationRps;
double rotPriority = rotationPriority.getAsDouble();
if (omegaFraction > rotPriority) {
    double translationScale = 1.0 - (omegaFraction - rotPriority);
    translationScale = Math.max(translationScale, 0.3); // minimum 30%
    vx *= translationScale;
    vy *= translationScale;
}
```

### Turbo/Sniper Compatibility
Turbo and sniper modes work correctly:
1. Speed modifiers are applied first (vx, vy multiplied)
2. Rotation priority then scales the modified speeds if needed
3. Robot slows down even in turbo mode to maintain heading

## Test Results (Before Final Changes)

| Speed | Heading Error |
|-------|---------------|
| 60% | 0.57° |
| 90% | 0.57° |
| 100% | 1.56° |

## Research Notes

Referenced implementations from top FRC teams:
- **Team 4738 Patribots**: ProfiledPIDController with P=5.75
- **Team 254 Cheesy Poofs**: Swerve Setpoint Generation
- **WPILib**: HolonomicDriveController with ProfiledPIDController

Our simpler approach (high P gain + rotation priority) achieves excellent results.

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - heading tracks well at all speeds

## Tuning Guide

**If rotation is too slow:**
- Increase `Orbit/Heading/kP` (e.g., 50-70)
- Decrease `Orbit/RotationPriority` (e.g., 0.3-0.4)

**If translation is too jerky:**
- Decrease `Orbit/Heading/kP` (e.g., 20-30)
- Increase `Orbit/RotationPriority` (e.g., 0.6-0.7)

**If robot oscillates around heading:**
- Decrease `Orbit/Heading/kP`
- Add small `Orbit/Heading/kD` value
