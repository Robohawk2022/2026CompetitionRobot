# Session: Replace Launcher/Agitator with HubbardShooter

**Date**: 2026-02-15

## Summary

Removed the Launcher (2-wheel closed-loop) and Agitator (feed motor) subsystems and replaced them with a fresh HubbardShooter subsystem — two NEO motors (CAN 51, 52) with 12:1 gear reduction (3:1 * 4:1), using simple open-loop voltage control.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/hubbardshooter/HubbardShooterHardware.java` - Hardware interface (applyVoltage, RPM, amps)
- `src/main/java/frc/robot/subsystems/hubbardshooter/HubbardShooterHardwareRev.java` - SparkMax implementation
- `src/main/java/frc/robot/subsystems/hubbardshooter/HubbardShooterHardwareSim.java` - Simulation with first-order dynamics
- `src/main/java/frc/robot/subsystems/hubbardshooter/HubbardShooterSubsystem.java` - Subsystem with intake/outtake/shoot commands
- `src/main/java/frc/robot/testbots/HubbardShooterTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Replaced `Launcher` + `Agitator` interfaces with `HubbardShooter`
- `src/main/java/frc/robot/Main.java` - Changed default testbot to `HubbardShooterTestbot`

### Files Deleted
- `src/main/java/frc/robot/subsystems/launcher/` - Entire directory (4 files)
- `src/main/java/frc/robot/subsystems/agitator/` - Entire directory (4 files)
- `src/main/java/frc/robot/testbots/LauncherTestbot.java`

## Config Changes

### Removed
- All `Launcher/*` preferences (wheel RPMs, PID gains, shot presets)
- All `Agitator/*` preferences (forward/feed power, invert, current limit)

### Added
- `HubbardShooter/IntakePower%` - default 20.0
- `HubbardShooter/OuttakePower%` - default 20.0
- `HubbardShooter/ShootingPower%` - default 50.0
- `HubbardShooter/Motor1Inverted?` - default false
- `HubbardShooter/Motor2Inverted?` - default false
- `HubbardShooter/CurrentLimit` - default 40.0

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `idleCommand()` | HubbardShooterSubsystem | Stops both motors (default command) |
| `intakeCommand()` | HubbardShooterSubsystem | Both motors same direction at intake power |
| `outtakeCommand()` | HubbardShooterSubsystem | Both motors reversed at outtake power |
| `shootCommand()` | HubbardShooterSubsystem | Motors counter-rotate at shooting power |
| `stopCommand()` | HubbardShooterSubsystem | Immediately stops both motors |

## Testing Done

- [x] `./gradlew build` - passed (all 70 tests pass)
- [ ] Simulation tested - not yet run
- [ ] Real hardware - not tested yet

## Notes for Next Session

- The HubbardShooter is NOT wired into RobotContainer yet — only available via HubbardShooterTestbot
- Motor inversion flags may need toggling once tested on real hardware
- Shooting power defaults to 50% — tune as needed
- The 12:1 gear ratio is documented but not used in calculations (open-loop voltage doesn't need it). It would matter if switching to velocity control later.
- The spelling is corrected from the old "Hubberd" to "Hubbard"
