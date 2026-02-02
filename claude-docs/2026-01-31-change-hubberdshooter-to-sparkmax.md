# Session: Change HubberdShooter from Falcon 500 to SparkMax NEO

**Date**: 2026-01-31

## Summary

Replaced the CTRE TalonFX (Falcon 500) hardware implementation with REV SparkMax (NEO) motors for the HubberdShooter subsystem.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareRev.java` - REV SparkMax implementation of the HubberdShooterHardware interface

### Files Modified
- `src/main/java/frc/robot/testbots/HubberdShooterTestbot.java` - Changed import and instantiation from CTRE to REV hardware
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardware.java` - Updated documentation to be motor-agnostic
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareSim.java` - Updated MAX_RPM from 6380 (Falcon 500) to 5676 (NEO)

### Files Deleted
- `src/main/java/frc/robot/subsystems/hubberdshooter/HubberdShooterHardwareCTRE.java` - Removed Falcon 500 implementation

## Implementation Details

### HubberdShooterHardwareRev.java

Key components:
- Two SparkMax motors (CAN IDs from Config: 51, 52)
- RelativeEncoder for velocity readings (returns RPM directly)
- SparkClosedLoopController for velocity control
- SparkMaxConfig for motor configuration

Key differences from CTRE implementation:

| Feature | TalonFX (CTRE) | SparkMax (REV) |
|---------|---------------|----------------|
| Velocity units | Rotations/sec | RPM (no conversion needed) |
| Voltage control | `VoltageOut` request | `motor.setVoltage()` |
| Velocity control | `VelocityVoltage` request | `closedLoopController.setReference()` |
| Current reading | `getStatorCurrent()` | `getOutputCurrent()` |
| PID config | `Slot0.kV/kP/kS` | `config.closedLoop.velocityFF/p` |
| kV conversion | Direct (V per rev/sec) | Divide by 60 (SparkMax expects V/RPM) |

Note: SparkMax doesn't have a direct kS equivalent in the closed-loop config, so static friction compensation is not applied.

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Real hardware - not tested yet (requires physical robot with NEO motors)

## Notes for Next Session

- SparkMax uses supply current limiting via `smartCurrentLimit()` rather than separate stator/supply limits
- May need to tune kV and kP values for NEO motors as they have different characteristics than Falcon 500s
- kS (static friction compensation) is not implemented in the REV version since SparkMax closed-loop doesn't have this feature
