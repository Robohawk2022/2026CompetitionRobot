# Session: Add Desired vs Actual Speed Telemetry to Swerve Dashboard

**Date**: 2026-03-14

## Summary

Added actual/measured speed telemetry alongside desired speeds on the swerve dashboard, using category-first naming for alphabetical grouping.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Renamed `SpeedX/SpeedY/SpeedOmega` to `SpeedXDesired/SpeedYDesired/SpeedOmegaDesired` and added `SpeedXCurrent/SpeedYCurrent/SpeedOmegaCurrent` entries using `getCurrentSpeeds()` (CTRE measured chassis speeds). All 6 entries are inside the `verboseLogging` block.

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

- `getCurrentSpeeds()` returns zeros in simulation since the sim lacks real motor feedback (dead reckoner handles pose). On real hardware, these values come from CTRE's odometry thread and should track the desired speeds closely.
