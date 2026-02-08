# Session: Update Launcher to 3-Motor Design

**Date**: 2026-02-08

## Summary

Removed the separate intake motor from the launcher. The bottom (lower) wheel now doubles as intake by spinning backward. Updated all hardware layers, config, subsystem, and testbot.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Removed `INTAKE_CAN_ID`, `intakePower`, `intakeInverted`. Added `intakeSpeedRPM` (RPM for lower wheel when pulling balls in backward). Now 3 CAN IDs: agitator=31, lower=32, upper=33.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` - Removed `applyIntakeVolts()`, `getIntakeRPM()`, `getIntakeAmps()`. Updated `stopAll()` for 3 motors.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` - Removed intake motor, encoder, and config. Now 3 motors.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` - Removed intake motor simulation state and methods. Now simulates 3 motors.
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` - Removed `intakeRPM` field and dashboard entries. Updated `intakeCommand()` to use lower wheel backward (negative RPM via velocity control). Updated `ejectCommand()` similarly. Removed intake hardware references from `shootCommand()` and `spinUpCommand()`.
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` - Simplified button mappings: A=intake, B=eject, X=shoot NEUTRAL, Y=stop. Removed HIGH_ARC/FLAT/spin-up bindings.

## Config Changes

- Removed `Launcher/IntakePower%`
- Removed `Launcher/IntakeInverted?`
- Removed `INTAKE_CAN_ID` (was 30)
- Added `Launcher/IntakeSpeedRPM` with default 1500.0

## Commands Added/Modified

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `intakeCommand()` | LauncherSubsystem | Lower wheel backward at IntakeSpeedRPM + agitator forward |
| `ejectCommand()` | LauncherSubsystem | Lower wheel backward at IntakeSpeedRPM + agitator reversed |
| `shootCommand()` | LauncherSubsystem | Both wheels forward at preset RPMs + agitator feeds (removed intake motor call) |
| `spinUpCommand()` | LauncherSubsystem | Both wheels forward, no agitator (removed intake motor call) |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not yet
- [ ] Real hardware - not tested yet

## Known Issues / TODO

- [ ] Velocity control currently runs in software (PDController at 50Hz). Should migrate to SparkMax onboard PID (1kHz) for better performance. This would change the hardware interface to use `setRPM()` instead of `applyVolts()`.

## Notes for Next Session

The intake now uses velocity control (via `driveWheels(-intakeSpeedRPM, 0)`) which goes through the same feedforward+PD path as shooting. The team wants to switch to SparkMax onboard PID for tuning — this would require refactoring the hardware interface to expose `setRPM()` methods and configuring `SparkMaxConfig.closedLoop` with kP/kD/kV.
