# Session: Fix HubberdShooter Shooting Mode

**Date**: 2026-01-31

## Summary

Changed shooting mode from velocity control (RPM-based) to voltage control (50% power) to match requirements spec.

## Requirements Review

| Mode | Requirement | Status |
|------|-------------|--------|
| Off | 0% power | Met |
| Intake | 20% power, synchronized | Met |
| Outtake | 20% power, synchronized | Met |
| Shooting | **50% power**, counter-rotate | **Now Met** (was using RPM) |

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java`
  - Removed: `shootingTargetRPM`, `kV`, `kP`, `kS` config parameters
  - Added: `shootingPower` (default 50%)

- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardware.java`
  - Removed: `setVelocity()`, `configurePid()` methods
  - Simplified interface to voltage control only

- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareCTRE.java`
  - Removed: `VelocityVoltage` request, PID configuration
  - Simplified to voltage-only control

- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareSim.java`
  - Removed: velocity mode tracking, PID gains
  - Simplified to voltage-only simulation

- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterSubsystem.java`
  - Changed `shootCommand()` to use voltage control at 50% power
  - Removed unused `rpmToRps()` helper
  - Updated dashboard to show `ShootingPower%` instead of `TargetRPM`

## Config Changes

| Parameter | Old Value | New Value |
|-----------|-----------|-----------|
| `HubberdShooter/ShootingTargetRPM` | 1200.0 | **Removed** |
| `HubberdShooter/kV` | 0.12 | **Removed** |
| `HubberdShooter/kP` | 0.1 | **Removed** |
| `HubberdShooter/kS` | 0.0 | **Removed** |
| `HubberdShooter/ShootingPower%` | N/A | **50.0** (new) |

## Motor Control Matrix (Updated)

| Mode | Motor 1 | Motor 2 | Power |
|------|---------|---------|-------|
| Off | 0V | 0V | 0% |
| Intake | +2.4V | +2.4V | 20% |
| Outtake | -2.4V | -2.4V | 20% |
| Shooting | +6V | -6V | 50% (counter-rotate) |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Notes for Next Session

All modes now use open-loop voltage control as specified in the requirements:
- Off: 0%
- Intake: 20% (configurable via `HubberdShooter/IntakePower%`)
- Outtake: 20% (configurable via `HubberdShooter/OuttakePower%`)
- Shooting: 50% (configurable via `HubberdShooter/ShootingPower%`)

The shooting mode applies +50% to motor 1 and -50% to motor 2 to achieve counter-rotation.
