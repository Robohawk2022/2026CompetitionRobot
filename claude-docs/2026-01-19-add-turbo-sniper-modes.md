# Session: Add Turbo and Sniper Speed Modes

**Date**: 2026-01-19

## Summary

Added turbo and sniper speed modes to the swerve drive, controlled by the driver's trigger axes. Left trigger activates sniper mode (slow, precise), right trigger activates turbo mode (fast).

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java`
  - Added `sniperFactor` config (default 0.25 = 25% speed)
  - Added `turboFactor` config (default 1.5 = 150% speed)
  - Added `applySniperToRotation` config (default true)

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Added `MathUtil` import
  - Added new `driveCommand()` overload that accepts trigger suppliers
  - Implements turbo/sniper speed multiplication with sniper priority

- `src/main/java/frc/robot/RobotContainer.java`
  - Added swerve subsystem instantiation
  - Set default command with turbo/sniper triggers mapped
  - Added binding: both sticks clicked = reset pose

## Config Changes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Swerve/SniperFactor` | 0.25 | Speed multiplier for sniper mode (25%) |
| `Swerve/TurboFactor` | 1.5 | Speed multiplier for turbo mode (150%) |
| `Swerve/SniperRotation?` | true | Apply sniper factor to rotation too |

## Control Mapping

| Control | Action |
|---------|--------|
| Left Stick | Translation (forward/strafe) |
| Right Stick X | Rotation |
| Left Trigger (>50%) | Sniper mode - slow, precise |
| Right Trigger (>50%) | Turbo mode - fast |
| Both Sticks Click | Reset pose |

## Implementation Details

- Trigger threshold is 0.5 (50%)
- Sniper mode takes priority if both triggers pressed
- Speed factors multiply the normalized joystick input before converting to m/s
- Values are clamped to [-1, 1] after multiplying to prevent overflow
- Dashboard mode string shows: `teleop-sniper`, `teleop-turbo`, or `teleop-field`

## Testing Done

- [x] `./gradlew clean build` - passed

## Known Issues / TODO

- [ ] Test in simulation with controller
- [ ] May want to tune default factors on real robot
- [ ] Consider adding deadzone config parameter

## Notes for Next Session

The turbo factor of 1.5 means the robot can exceed normal max speed when turbo is held. This is intentional but may need tuning based on driver preference. The factors are tunable via Preferences/NetworkTables at runtime.
