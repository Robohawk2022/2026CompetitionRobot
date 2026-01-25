# Session: Add IntakeFront Subsystem

**Date**: 2026-01-25

## Summary

Created a front intake subsystem with configurable motor direction for sucking in game pieces.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardware.java` - Hardware interface
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardwareSparkMax.java` - SparkMax implementation
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardwareSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontSubsystem.java` - Main subsystem with commands
- `src/main/java/frc/robot/testbots/IntakeFrontTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added IntakeFront configuration interface

## Config Changes

Added `Config.IntakeFront` interface with:
- `MOTOR_CAN_ID` = 20 (update to match your CAN ID)
- `IntakeFront/SpeedPercent` = 100.0 (0-100%)
- `IntakeFront/Inverted?` = false (toggle if motor spins wrong direction)

## Commands Added

| Command | Description |
|---------|-------------|
| `intakeCommand()` | Runs intake to suck in (hold to run) |
| `ejectCommand()` | Runs intake in reverse to eject (hold to run) |
| `stopCommand()` | Stops the intake motor |
| `idleCommand()` | Default idle state |

## Direct Control Methods

For use outside the command system:
- `runIntake()` - Start intaking
- `runEject()` - Start ejecting
- `stop()` - Stop the motor

## Testing

Run the testbot:
```bash
./gradlew simulateJava -Probot=IntakeFrontTestbot
```

Button mappings:
- **A (hold)** - Intake (suck in)
- **B (hold)** - Eject (spit out)

## How to Configure Direction

1. Run the testbot or deploy to robot
2. Press A to run intake
3. If the motor spins the wrong direction (ejecting instead of intaking):
   - Open Elastic/Shuffleboard
   - Find `IntakeFront/Inverted?`
   - Toggle it to `true`
4. The change takes effect immediately (uses Preferences)

## Wiring in RobotContainer

```java
// Create intake
IntakeFrontSubsystem intake = new IntakeFrontSubsystem(
    Robot.isSimulation() ? new IntakeFrontHardwareSim() : new IntakeFrontHardwareSparkMax());

// Set default command
intake.setDefaultCommand(intake.idleCommand());

// Bind to button
operator.a().whileTrue(intake.intakeCommand());
operator.b().whileTrue(intake.ejectCommand());
```

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Notes for Next Session

- Update `MOTOR_CAN_ID` in Config.java to match actual CAN ID
- Test direction on real robot and set `Inverted?` preference accordingly
- May want to add current limiting or stall detection
