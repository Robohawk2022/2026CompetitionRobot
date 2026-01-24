# Session: Orbit Turbo Mode Fix

**Date**: 2026-01-23

## Summary

Modified orbit turbo mode to use 90% of the swerve drive max speed instead of applying the turboFactor multiplier to orbit-specific speeds.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` - Added `Orbit/TurboSpeedFraction` parameter
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Modified turbo handling in orbitCommand

## Config Parameters Added

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Orbit/TurboSpeedFraction` | 0.9 | Turbo speed as fraction of swerve max speed |
| `Orbit/RotationTrimAuthority` | 0.5 | Rotation trim as fraction of max rotation (was hardcoded 0.3) |

## Behavior Change

**Before:**
- Orbit turbo applied `turboFactor` (1.5x) to orbit speeds
- Result: radial = 12 FPS, tangent = 15 FPS

**After:**
- Orbit turbo uses fraction of swerve max speed
- Result: both radial and tangent = 13.5 FPS (90% of 15 FPS)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation testing - not performed

## Notes for Next Session

The turbo speed fraction is tunable via Preferences if the 90% value needs adjustment.
