# Session: Add Amperage Logging + Fix Shooter kV

**Date**: 2026-03-01

## Summary

Added motor amperage logging to all subsystems (shooter, ball path, swerve drive) for diagnosing battery-related performance issues. Also fixed shooter feedforward (kV) being 10x too low, which caused jerky RPM response when shooting.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardware.java` - Added `getShooterAmps()` to interface
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardwareRev.java` - Implemented `getShooterAmps()` via SparkMax `getOutputCurrent()`
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardwareSim.java` - Stub returning 0.0
- `src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java` - Added `ShooterMotor/Amps` to dashboard
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardware.java` - Added `getIntakeAmps()`, `getFeederAmps()`, `getAgitatorAmps()`
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardwareRev.java` - Implemented via SparkMax `getOutputCurrent()`
- `src/main/java/frc/robot/subsystems/ballpath/BallPathHardwareSim.java` - Stubs returning 0.0
- `src/main/java/frc/robot/subsystems/ballpath/BallPathSubsystem.java` - Added `IntakeMotor/Amps`, `FeederMotor/Amps`, `AgitatorMotor/Amps` to dashboard
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added per-module `DriveAmps/XX` and `SteerAmps/XX` (FL, FR, BL, BR) via TalonFX `getSupplyCurrent()`
- `src/main/java/frc/robot/Config.java` - Changed shooter kV from 0.0002 to 0.002, kP from 0.001 to 0.0003

## Config Changes

- `ShooterSubsystem/ShooterMotor/kV` default changed from 0.0002 to 0.002
- `ShooterSubsystem/ShooterMotor/kP` default changed from 0.001 to 0.0003

## Dashboard Keys Added

| Key | Subsystem | Description |
|-----|-----------|-------------|
| `ShooterMotor/Amps` | ShooterSubsystem | Shooter motor current draw |
| `IntakeMotor/Amps` | BallPathSubsystem | Intake motor current draw |
| `FeederMotor/Amps` | BallPathSubsystem | Feeder motor current draw |
| `AgitatorMotor/Amps` | BallPathSubsystem | Agitator motor current draw |
| `DriveAmps/FL,FR,BL,BR` | SwerveSubsystem | Per-module drive motor supply current |
| `SteerAmps/FL,FR,BL,BR` | SwerveSubsystem | Per-module steer motor supply current |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Real hardware - not tested yet

## Notes for Next Session

- The kV and kP values are tunable via Preferences - adjust on the real robot
- kV = 0.002 is a starting point based on NEO free speed (~5676 RPM at 12V)
- If shooter still oscillates, try lowering kP further
- If spin-up is too slow, try increasing kV slightly
- Watch the Amps values on dashboard to correlate battery voltage with shooter accuracy
