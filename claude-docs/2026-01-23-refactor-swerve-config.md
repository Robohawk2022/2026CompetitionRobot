# Session: Refactor Swerve Config to Reduce Preference Overuse

**Date**: 2026-01-23

## Summary

Analyzed Config Preferences usage and moved hardware-specific swerve constants out of Preferences into a new `SwerveHardwareConfig` class. These values are only read at startup so Preferences provided no benefit.

## Problem

Several Config.Swerve preferences were being read once in constructors and couldn't be updated live:
- Motor PID gains (driveKP, driveKV, driveKS, turnKP, turnKD) - written to TalonFX motor controllers at startup
- CAN IDs, angular offsets, motor inversions - used once during hardware initialization

Making these Preferences was misleading since changes wouldn't take effect until robot code was redeployed.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareConfig.java` - New class containing:
  - All CAN IDs (FL/FR/BL/BR drive, turn, encoder + Pigeon)
  - Angular offsets for each module
  - Motor inversion flags
  - Drive PID constants (DRIVE_KP, DRIVE_KV, DRIVE_KS)
  - Turn PID constants (TURN_KP, TURN_KD)

### Files Modified
- `src/main/java/frc/robot/Config.java`:
  - Removed CAN IDs, angular offsets, inversion flags
  - Removed PID Preferences (driveKP, driveKV, driveKS, turnKP, turnKD)
  - Removed CTRE InvertedValue import
  - Kept tunable Preferences that work live (maxSpeedFps, sniperFactor, etc.)
  - Moved chassis dimensions (WHEEL_BASE_INCHES, TRACK_WIDTH_INCHES) to physical constants section

- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java`:
  - Changed import from `Config.Swerve` to `SwerveHardwareConfig`

- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`:
  - Added import for `SwerveHardwareConfig`
  - Updated PID configuration to use static constants instead of Preference suppliers

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

The following Logging preferences were identified as dead code (defined but never used):
- `Logging/Vision?` (visionLogging)
- `Logging/Odometry?` (odometryLogging)
- `Logging/Shooter?` (shooterLogging)

These should either be implemented or removed in a future session.
