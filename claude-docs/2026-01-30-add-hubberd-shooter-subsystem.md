# Session: Add HubberdShooter Subsystem

**Date**: 2026-01-30

## Summary

Created a new HubberdShooter subsystem for testing two Falcon 500 motors with four operating modes: Off, Intake, Outtake, and Shooting.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardware.java` - Hardware interface defining motor control methods
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareCTRE.java` - TalonFX implementation using Phoenix 6
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareSim.java` - Simulation with first-order dynamics
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterSubsystem.java` - Main subsystem with command factories
- `src/main/java/frc/robot/testbots/HubberdShooterTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `HubberdShooter` interface with all config parameters

## Config Changes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HubberdShooter/IntakePower%` | 20.0 | Power for intake mode (0-100%) |
| `HubberdShooter/OuttakePower%` | 20.0 | Power for outtake mode (0-100%) |
| `HubberdShooter/ShootingTargetRPM` | 1200.0 | Target RPM for shooting mode |
| `HubberdShooter/kV` | 0.12 | Velocity feedforward gain |
| `HubberdShooter/kP` | 0.1 | Proportional gain |
| `HubberdShooter/kS` | 0.0 | Static friction compensation |
| `HubberdShooter/Motor1Inverted?` | false | Invert motor 1 direction |
| `HubberdShooter/Motor2Inverted?` | false | Invert motor 2 direction |
| `HubberdShooter/StatorCurrentLimit` | 60.0 | Stator current limit (amps) |
| `HubberdShooter/SupplyCurrentLimit` | 40.0 | Supply current limit (amps) |

CAN IDs: Motor 1 = 51, Motor 2 = 52

## Commands Added

| Command | Description |
|---------|-------------|
| `stopCommand()` | Instant stop - sets both motors to 0V |
| `idleCommand()` | Default command - keeps motors stopped |
| `intakeCommand()` | Both motors same direction at intake power |
| `outtakeCommand()` | Both motors same direction (reversed) at outtake power |
| `shootCommand()` | Velocity control - motors counter-rotate at target RPM |

## Motor Control Matrix

| Mode | Motor 1 | Motor 2 | Control Type |
|------|---------|---------|--------------|
| Off | 0V | 0V | Voltage |
| Intake | +2.4V | +2.4V | Voltage (20% = 2.4V) |
| Outtake | -2.4V | -2.4V | Voltage |
| Shooting | +10 RPS | +10 RPS* | Velocity |

*Motor 2 is inverted in hardware config, so +10 RPS command results in counter-rotation

## Testbot Button Mapping

| Button | Action | Mode |
|--------|--------|------|
| A (hold) | Intake | Both motors pull fuel in at 20% |
| B (hold) | Outtake | Both motors push fuel out at 20% |
| X (hold) | Shoot | Counter-rotate at 1200 RPM |
| Y (press) | Stop | All motors stop |

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - all modes working correctly:
  - Intake (A): Both motors positive RPM ✓
  - Outtake (B): Both motors negative RPM ✓
  - Shoot (X): Motors counter-rotate (Motor1 +, Motor2 -) ✓
- [ ] Real hardware - not tested yet

## Design Notes

1. **Naming**: Used `HubberdShooter` to distinguish from existing `ShooterSubsystem` (flywheel-based)

2. **Control strategy**:
   - Intake/Outtake use open-loop voltage control for simplicity
   - Shooting uses closed-loop velocity control via TalonFX internal PID

3. **Motor inversion**: Neither motor is inverted by default. The shooting command explicitly sends opposite velocities (`+RPS` to motor 1, `-RPS` to motor 2) to achieve counter-rotation. This allows intake/outtake modes to work correctly with same-direction commands

4. **Hardware interface**: Separates voltage control (`applyVoltage`) from velocity control (`setVelocity`) to make the mode explicit

## Known Issues / TODO

- [ ] Tune kV and kP on real hardware
- [ ] Verify counter-rotation direction is correct for fuel propulsion
- [ ] May need to adjust current limits based on motor heating

## Notes for Next Session

To test in simulation:
```bash
./gradlew simulateJava -Probot=HubberdShooterTestbot
```

Watch the dashboard for:
- `HubberdShooterSubsystem/Motor1RPM`
- `HubberdShooterSubsystem/Motor2RPM`
- `HubberdShooterSubsystem/Mode`

The simulation uses first-order dynamics with ~50ms time constant, so motors will ramp to speed rather than instantly reaching target.
