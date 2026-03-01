# Session: Split LauncherSubsystem into ShooterSubsystem + BallPathSubsystem

**Date**: 2026-02-28

## Summary

Split the monolithic `LauncherSubsystem` (4 motors) into two independent subsystems: `ShooterSubsystem` (1 shooter motor) and `BallPathSubsystem` (intake, feeder, agitator). This enables independent commands and cleaner coordination via WPILib command composition.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardware.java` — hardware interface for single shooter motor
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardwareRev.java` — SparkMax implementation (extracted from LauncherHardwareRev)
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardwareSim.java` — simulation (stores RPM)
- `src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java` — subsystem with coast/velocityCommand/spinUpCommand/shooterAtSpeed
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardware.java` — hardware interface for 3 ball-path motors
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardwareRev.java` — 3x SparkMax implementation (extracted from LauncherHardwareRev)
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardwareSim.java` — simulation (stores 3 RPMs)
- `src/main/java/frc/robot/subsystems/ballpath/BallPathSubsystem.java` — subsystem with coast/velocityCommand/intakeCommand/ejectCommand/feedCommand

### Files Modified
- `src/main/java/frc/robot/Config.java` — replaced `LauncherSpeeds`/`Launcher` with `BallPathSpeeds`, `Shooter`, `BallPath`, `BallHandling` interfaces
- `src/main/java/frc/robot/RobotContainer.java` — replaced `launcher` with `shooter` + `ballPath`, updated button bindings
- `src/main/java/frc/robot/commands/ShootingCommands.java` — `driveAndShootCommand` now takes `shooter` + `ballPath`; updated imports
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` — uses `shooter` + `ballPath` instead of `launcher`
- `src/main/java/frc/robot/testbots/ShootingSimTestbot.java` — uses `shooter` + `ballPath` instead of `launcher`

### Files Deleted
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java`
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java`
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java`
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java`

## Config Changes

### Removed
- `LauncherSpeeds` class (had 4 motor RPMs)
- `interface Launcher` (combined all 4 motors)

### Added
- `BallPathSpeeds` class (3 motor RPMs: intake, feeder, agitator)
- `interface Shooter` — `shooterPid`, `shootRpm` (3000), `shootSpeedTolerance` (100), `stallSpeed` (30), `stallTime` (1.0)
- `interface BallPath` — `intakePid`, `feederPid`, `agitatorPid`, `intakeSpeeds`, `ejectSpeeds`, `feedSpeeds`, `stallSpeed`, `stallTime`
- `interface BallHandling` — `shootDistanceFeet`, `shootDistanceTolerance`, `shootSpinupTime` (used by ShootingCommands)

### Preference path changes
- PID: `LauncherSubsystem/XxxMotor/*` → `ShooterSubsystem/ShooterMotor/*` and `BallPathSubsystem/XxxMotor/*`
- Speeds: `BallHandling/IntakeSpeeds/*` (kept), `BallHandling/EjectSpeeds/*` (kept), new `BallHandling/FeedSpeeds/*`
- Stall: `BallHandling/StallSpeed` → separate `Shooter/StallSpeed` and `BallPath/StallSpeed`
- Shoot RPM: `BallHandling/ShootSpeeds/ShooterRpm` → `Shooter/ShootRpm`

## Commands Added/Modified

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `coast()` | ShooterSubsystem | Stops the shooter motor |
| `velocityCommand(rpm)` | ShooterSubsystem | Runs shooter at specific RPM |
| `spinUpCommand()` | ShooterSubsystem | Spins shooter to configured shoot RPM |
| `coast()` | BallPathSubsystem | Stops all ball-path motors |
| `velocityCommand(i, f, a)` | BallPathSubsystem | Runs 3 motors at specific RPMs |
| `intakeCommand()` | BallPathSubsystem | Runs ball-path at intake speeds |
| `ejectCommand()` | BallPathSubsystem | Runs ball-path at eject speeds (reversed) |
| `feedCommand()` | BallPathSubsystem | Runs ball-path at feed speeds (for shooting) |

## Button Bindings (RobotContainer)

| Button | Old | New |
|--------|-----|-----|
| A | `launcher.intakeCommand()` | `ballPath.intakeCommand()` |
| B | `launcher.shootCommand()` | `parallel(shooter.spinUpCommand(), ballPath.feedCommand())` |
| X | `launcher.ejectCommand()` | `ballPath.ejectCommand()` |
| Y | `launcher.coast()` | `parallel(shooter.coast(), ballPath.coast())` |
| Right bumper | `driveAndShootCommand(swerve, launcher)` | `driveAndShootCommand(swerve, shooter, ballPath)` |

## Testing Done

- [x] `./gradlew build` - passed (all tests pass)
- [ ] Simulation tested - not run this session
- [ ] Real hardware - not tested

## Notes for Next Session

- `MotorStatus` inner class is duplicated in both ShooterSubsystem and BallPathSubsystem (~40 lines each). This is intentional — self-contained and not worth a shared base class.
- `createMotor()` helper is duplicated in both Rev hardware classes (small private method).
- `feedCommand()` is new — runs intake/feeder/agitator at shoot-mode RPMs to feed balls into the spinning shooter. Previously this was handled by `shootCommand()` which also controlled the shooter motor.
- The shooter no longer spins during intake/eject modes (previously `intakeSpeeds.shooterRpm` was 1000).
