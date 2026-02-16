# Session: Add Flapper Motor and Distance-Based Shooting

**Date**: 2026-02-16

## Summary

Added a 4th motor (flapper) to the launcher subsystem for keeping balls from re-entering during intake. Also added distance-based shooter RPM using an interpolation table with pose estimation from Limelight/odometry.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` — Added flapper PID (kV/kP/kD), FLAPPER_RPM constant, distance-based shooting config (CLOSE_DISTANCE_FEET=2.5, CLOSE_RPM=2525, closeRangeFeet, mediumRangeFeet thresholds)
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` — Added setFlapperRPM, getFlapperRPM, getFlapperAmps, resetFlapperPID; updated stopAll() to include flapper
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` — Added 4th SparkMax for flapper (FLAPPER_INVERTED=false), constructor takes 4th CAN ID
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` — Added 4th simulated flywheel
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` — Added flapper to driveMotors/cleanup/atSpeed/applyPIDGains. Added pose supplier (optional), InterpolatingDoubleTreeMap for distance→RPM, getDistanceToHopper(), getTargetRPMForDistance(), autoShootCommand(). Dashboard shows DistanceToHopper.
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` — Added FLAPPER_CAN_ID=30, passed to hardware constructor. Fixed currentMode access to use getter.
- `src/main/java/frc/robot/subsystems/led/LEDSignal.java` — Added INTAKING_ACTIVE signal (previous session, carried over)

## Config Changes

- Added `Launcher/Flapper/kV` (default 0.00017)
- Added `Launcher/Flapper/kP` (default 0.0004)
- Added `Launcher/Flapper/kD` (default 0.0)
- Added `FLAPPER_RPM = 2000.0` constant
- Added `CLOSE_DISTANCE_FEET = 2.5` constant
- Added `CLOSE_RPM = 2525.0` constant
- Added `Launcher/CloseRangeFeet` (default 5.0)
- Added `Launcher/MediumRangeFeet` (default 10.0)

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `autoShootCommand()` | LauncherSubsystem | Shoots with distance-based RPM from interpolation table; falls back to fixed RPM without pose |

## Testing Done

- [x] `./gradlew build` — passed
- [ ] Real hardware — not tested yet
- [ ] Distance-based shooting — needs swerve wiring to test

## Known Issues / TODO

- [ ] Flapper needs PID tuning on real hardware
- [ ] Flapper inversion may need flipping (defaulted to false)
- [ ] Only 1 data point in distance→RPM table (2.5ft → 2525 RPM) — need more from practice field
- [ ] LED distance-based signals (SHOOT_RANGE_CLOSE/MEDIUM/FAR) exist but aren't wired yet
- [ ] Launcher still not wired into RobotContainer (only in testbot)
- [ ] Staged shoot sequence (spin up shooter first, then feed) not yet implemented
- [ ] autoShootCommand not bound to any button in testbot yet

## Notes for Next Session

- Flapper CAN ID: 30, 12:1 gearbox NEO on SparkMax
- `InterpolatingDoubleTreeMap` is used for distance→RPM lookup; add more data points with `distanceToRPM.put(distFeet, rpm)`
- With only 1 data point, interpolation returns 2525 RPM for all distances — need at least 2 points for meaningful interpolation
- `autoShootCommand()` uses distance-based RPM when pose supplier is available, falls back to Config `shooterRPM` otherwise
- The user also discussed a staged shoot sequence (shooter + feeder right first, then feeder left) — not implemented yet, will be needed when balls are pre-loaded
