# Session: Rebuild Launcher for New 3-Motor Design

**Date**: 2026-02-16

## Summary

Rebuilt the launcher subsystem from a 2-wheel + agitator design to a new 3-motor design: 2 feeder motors (bottom, facing each other) + 1 shooter motor (top). Deleted the agitator subsystem entirely.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` — Replaced `Launcher` interface (removed shot presets, upper/lower wheel config; added feeder/shooter CAN IDs, shared feeder PID, separate shooter PID, RPM targets). Removed `Agitator` interface entirely.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` — Rewritten for 3 motors: `setFeederLeftRPM`, `setFeederRightRPM`, `setShooterRPM` + getters + PID reset methods.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` — 3 SparkMax motors (CAN 32=feeder left, CAN 11=feeder right, CAN 60=shooter). Shared PID for feeders, separate PID for shooter.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` — 3 simulated flywheels with first-order dynamics.
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` — Removed `ShotPreset` enum. New commands: `intakeCommand()`, `ejectCommand()`, `shootCommand()`, `idleCommand()`, `stopCommand()`.
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` — Simplified to single subsystem, removed agitator references.

### Files Deleted
- `src/main/java/frc/robot/subsystems/agitator/AgitatorSubsystem.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardware.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareRev.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareSim.java`

## Config Changes

### Removed
- All `Launcher/Lower/*`, `Launcher/Upper/*` PID params
- All shot preset RPMs (`Launcher/HighArc/*`, `Launcher/Flat/*`, `Launcher/Neutral/*`)
- `Launcher/IntakeLowerRPM`, `Launcher/IntakeUpperRPM`, `Launcher/EjectSpeedRPM`
- `Launcher/LowerWheelInverted?`, `Launcher/UpperWheelInverted?`
- Entire `Agitator/*` config

### Added
- CAN IDs: `FEEDER_LEFT_CAN_ID = 32`, `FEEDER_RIGHT_CAN_ID = 11`, `SHOOTER_CAN_ID = 60`
- `FEEDER_GEAR_RATIO = 3.0`
- `Launcher/Feeder/kV` (default 0.00017), `Launcher/Feeder/kP` (default 0.0004)
- `Launcher/Shooter/kV` (default 0.00017), `Launcher/Shooter/kP` (default 0.0004)
- `Launcher/FeederRPM` (default 2000), `Launcher/FeedShootRPM` (default 3000), `Launcher/ShooterRPM` (default 4500)
- `Launcher/FeederLeftInverted?` (default false), `Launcher/FeederRightInverted?` (default true), `Launcher/ShooterInverted?` (default false)
- `Launcher/ToleranceRPM` (default 100)

## Commands Added/Modified

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `idleCommand()` | LauncherSubsystem | Stops all 3 motors (default) |
| `intakeCommand()` | LauncherSubsystem | Feeders spin inward at intake RPM, shooter off |
| `ejectCommand()` | LauncherSubsystem | Feeders spin outward (reverse), shooter off |
| `shootCommand()` | LauncherSubsystem | Feeders at feed RPM + shooter at shoot RPM |
| `stopCommand()` | LauncherSubsystem | Immediate stop |

## Testing Done

- [x] `./gradlew build` — passed (all tests green)
- [x] No remaining agitator references in Java code
- [ ] Simulation tested — not yet run
- [ ] Real hardware — not tested yet

## Known Issues / TODO

- [ ] PID gains are placeholder defaults — need tuning on real hardware
- [ ] RPM targets are placeholder — need tuning
- [ ] Feeder inversion flags need verification on real robot (left=false, right=true assumes right needs inverting to spin inward)
- [ ] Shooter inversion needs verification
- [ ] Launcher not yet wired into RobotContainer (still only in testbot)

## Notes for Next Session

- CAN IDs: Feeder Left = 32, Feeder Right = 11 (same side as shooter, spins inward), Shooter = 60
- Feeder right is on the same side as the shooter motor
- Both feeders use shared PID gains since they're the same mechanism
- The `atSpeed()` check only validates motors with non-zero targets
