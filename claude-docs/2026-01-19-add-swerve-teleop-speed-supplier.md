# Session: Add SwerveTeleopSpeedSupplier

**Date**: 2026-01-19

## Summary

Created a `SwerveTeleopSpeedSupplier` class to encapsulate teleop joystick input handling and simplified the RobotContainer controller setup.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - New class that encapsulates:
  - Left stick Y for forward/backward (inverted)
  - Left stick X for strafe (inverted)
  - Right stick X for rotation (inverted)
  - Left trigger for sniper mode
  - Right trigger for turbo mode
  - Applies deadband from `Config.Swerve.deadzone`
  - Provides `driveCommand()` convenience method

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `deadzone` config parameter to Swerve interface
- `src/main/java/frc/robot/RobotContainer.java` - Updated to use `SwerveTeleopSpeedSupplier`:
  - Removed hardcoded `DEADZONE` constant
  - Removed `MathUtil` import (no longer needed)
  - Added `SwerveTeleopSpeedSupplier` import
  - Replaced inline supplier setup with cleaner two-line initialization

## Config Changes

- Added `Swerve/Deadzone` with default value 0.1

## Standard Controller Bindings (unchanged)

| Control | Action |
|---------|--------|
| Left Stick | Translation (forward/strafe) |
| Right Stick X | Rotation |
| Left Trigger | Sniper mode (slow, precise) |
| Right Trigger | Turbo mode (fast) |
| Both Sticks Click | Reset pose/heading |

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

The `SwerveTeleopSpeedSupplier` class can be extended with additional features like:
- Different driving profiles (arcade vs tank)
- Exponential input scaling for finer control at low speeds
- Operator-specific supplier for a second driver if needed
