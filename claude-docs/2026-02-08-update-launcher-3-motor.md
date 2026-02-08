# Session: Update Launcher to 3-Motor Design

**Date**: 2026-02-08

## Summary

Refactored the launcher from 4 motors to 3. Removed separate intake motor (lower wheel doubles as intake). Split agitator into its own subsystem. Migrated wheel velocity control to SparkMax onboard PID with separate kV/kP per wheel (no kD). Fixed critical `velocityFF` units bug and eject direction bug during code review.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardware.java` - Hardware interface for single agitator motor (open-loop)
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareRev.java` - REV SparkMax implementation
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/agitator/AgitatorSubsystem.java` - Agitator subsystem with forward/reverse/feed/idle commands
- `claude-docs/launcher-guide.md` - Launcher testing and PID tuning guide

### Files Modified
- `src/main/java/frc/robot/Config.java` - Removed intake motor config. Split wheel PID into per-motor (Lower/kV, Lower/kP, Upper/kV, Upper/kP). Removed kD. Added Agitator config section (CAN_ID=31, ForwardPower%, FeedPower%, Inverted?, CurrentLimit).
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` - Wheels only. Removed agitator methods. resetPID split into resetLowerPID/resetUpperPID (kV+kP only, no kD).
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` - Wheels only. Uses `feedForward.kV()` (not deprecated `velocityFF()`). Separate config per wheel.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` - Wheels only.
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` - Wheels only. Commands: idleCommand, intakeWheelCommand, ejectWheelCommand, spinUpCommand, stopCommand. No agitator references.
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` - Wires both subsystems. A=intake (parallel wheel+agitator), B=eject (parallel), X=shoot (spin up, waitUntil atSpeed, then feed), Y=stop.

## Code Review Findings (Fixed)

1. **CRITICAL: `velocityFF()` wrong units** - Was duty cycle/RPM, switched to `feedForward.kV()` which is volts/RPM
2. **CRITICAL: kP default too high** - Changed from 0.001 to 0.0001 for duty cycle units
3. **BUG: Eject direction** - Lower wheel was going same direction as intake, fixed to go forward

## Config Layout

### Launcher (wheels)
- `Launcher/IntakeSpeedRPM` = 1500
- `Launcher/Lower/kV` = 0.002, `Launcher/Lower/kP` = 0.0001
- `Launcher/Upper/kV` = 0.002, `Launcher/Upper/kP` = 0.0001
- `Launcher/Wheels/ToleranceRPM` = 100
- `Launcher/LowerWheelInverted?`, `Launcher/UpperWheelInverted?`
- `Launcher/CurrentLimit` = 40
- Shot presets: HighArc, Flat, Neutral RPMs

### Agitator
- `Agitator/ForwardPower%` = 30, `Agitator/FeedPower%` = 50
- `Agitator/Inverted?`, `Agitator/CurrentLimit` = 40
- CAN ID = 31

## Shoot Command Pattern

Wheels and agitator are separate subsystems, composed for shooting:
```java
Commands.parallel(
    launcher.spinUpCommand(preset),        // wheels spin immediately
    Commands.sequence(
        Commands.waitUntil(launcher::atSpeed), // wait for target RPM
        agitator.feedCommand()                 // then start feeding
    )
)
```
Wheels never stop between spin-up and feed.

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - intake, eject, shoot all working correctly
- [ ] Real hardware - not tested yet

## Notes for Next Session

- `IntakeFrontHardwareSparkMax` still uses deprecated `velocityFF()` API — should migrate to `feedForward.kV()`
- Test motor directions on real robot using Inverted preferences
- Tune kV first (per wheel), then kP
- launcher-guide.md has full tuning procedure
