# Session: Add TeleopInput Helper Class

**Date**: 2026-01-27

## Summary

Created `TeleopInput` utility class to centralize shared teleop input logic between `SwerveTeleopCommand` and `SwerveOrbitCommand`. Added sniper/turbo mode support to orbit command.

## Changes Made

### Files Created
- `src/main/java/frc/robot/util/TeleopInput.java` - Helper class with:
  - `TurboSniperMode` enum (SNIPER, TURBO, NORMAL) with instance methods:
    - `getSpeedFactor()` - returns speed multiplier (sniper/turbo/1.0)
    - `applyFactorToRotation()` - whether to apply factor to rotate-in-place
  - `getMode(GameController)` - detects mode from trigger state
  - `conditionInput(double)` - applies deadband and exponent

### Files Modified
- `src/main/java/frc/robot/commands/swerve/SwerveTeleopCommand.java`
  - Removed local `Mode` enum, `getMode()`, and `conditionInput()` methods
  - Now uses `TeleopInput` for input conditioning and mode detection
  - Uses `getSpeedFactor()` for translation, `applyFactorToRotation()` to conditionally apply to rotation

- `src/main/java/frc/robot/commands/swerve/SwerveOrbitCommand.java`
  - Added sniper/turbo mode support using `TeleopInput`
  - Now uses `conditionInput()` instead of just deadband (adds exponent)
  - Dashboard now shows Mode
  - Applies speed factor to radial, tangential, and rotation speeds (orbit rotation is tied to tangential motion, not rotate-in-place)

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

The orbit command now applies the exponent to joystick input (via `conditionInput`), which it didn't before. This might change the feel slightly - it previously only applied deadband. If drivers prefer the old linear response for orbit, could add a config flag or a separate `conditionInputLinear()` method that only applies deadband.
