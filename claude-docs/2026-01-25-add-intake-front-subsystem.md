# Session: Add IntakeFront Subsystem

**Date**: 2026-01-25

## Summary

Created a front intake subsystem with closed-loop velocity control, separate intake/eject speeds, and stall detection for hopper-full alerts.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardware.java` - Hardware interface
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardwareSparkMax.java` - SparkMax implementation with closed-loop velocity control
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontHardwareSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/intakefront/IntakeFrontSubsystem.java` - Main subsystem with commands and stall detection
- `src/main/java/frc/robot/testbots/IntakeFrontTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added IntakeFront configuration interface

## Config Changes

Added `Config.IntakeFront` interface with:

### Speed Settings
- `IntakeFront/IntakeSpeedRPS` = 80.0 (target intake speed in rev/sec)
- `IntakeFront/EjectSpeedRPS` = 60.0 (target eject speed in rev/sec)
- `IntakeFront/Inverted?` = false (toggle if motor spins wrong direction)

### Closed-Loop PID Gains
- `IntakeFront/kV` = 0.12 (velocity feedforward in V/(rev/s))
- `IntakeFront/kP` = 0.1 (proportional gain)

### Stall Detection
- `IntakeFront/StallThresholdRPM` = 100.0 (velocity below this = stalled)
- `IntakeFront/StallTimeSec` = 0.25 (time before stall triggers)

### Motor Limits
- `IntakeFront/CurrentLimit` = 40.0 (amps)
- `MOTOR_CAN_ID` = 20 (update to match your CAN ID)

## Commands Added

| Command | Description |
|---------|-------------|
| `intakeCommand()` | Runs intake at IntakeSpeedRPS (hold to run) |
| `ejectCommand()` | Runs intake in reverse at EjectSpeedRPS (hold to run) |
| `stopCommand()` | Stops the intake motor |
| `idleCommand()` | Default idle state |

## Stall Detection

The subsystem monitors for stalls (low velocity while commanding motor). When stalled:
- `isStalled()` returns `true`
- Dashboard shows `Stalled? = true`
- Optional callback fires for LED integration

### Wiring Stall to LEDs

```java
// In RobotContainer
intake.setStallCallback(stalled -> {
    if (stalled) {
        led.setColor(LEDColor.RED);  // Hopper full!
    } else {
        led.setColor(LEDColor.OFF);
    }
});
```

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

// Optional: wire stall detection to LEDs
intake.setStallCallback(stalled -> {
    if (stalled) {
        System.out.println("Hopper full!");
        // led.setColor(LEDColor.RED);
    }
});
```

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Code Review Fixes Applied

### Initial Implementation
1. **Added `disabledInit()` to testbot** - Motors now stop when robot is disabled
2. **Added voltage clamping in hardware layer** - Both SparkMax and Sim implementations now clamp to ±12V
3. **Fixed `idleCommand()` efficiency** - Changed from `run()` to `startRun()` so `stop()` is only called once on initialization
4. **Replaced magic number 12.0 with `Util.MAX_VOLTS`** - Consistent with rest of codebase
5. **Added speed percentage validation** - Clamped to 0-100 via `getSpeedFactor()` helper
6. **Made current limit configurable** - Added `IntakeFront/CurrentLimit` preference
7. **Extracted cleanup code** - Created `cleanup()` helper method
8. **Made `stop()` a default method** - In `IntakeFrontHardware` interface
9. **Removed empty override methods** - Cleaned up testbot
10. **Added warning comments** - Direct control methods now have safety warnings

### Second Review - Major Enhancements
11. **Separate intake/eject speeds** - Added `IntakeSpeedRPS` and `EjectSpeedRPS` config values
12. **Closed-loop velocity control** - Changed from open-loop voltage to closed-loop velocity PID
13. **Modified hardware interface** - Replaced `applyVolts()` with `setSpeed()` and added `resetPid()`
14. **SparkMax closed-loop** - Uses onboard velocity PID with feedforward
15. **Stall detection** - Monitors for low velocity + commanded output over time
16. **Stall callback** - Added `setStallCallback()` for LED integration
17. **PID reset in commands** - `resetPid()` called in command `initialize()` to pick up current gains

## Notes for Next Session

- Update `MOTOR_CAN_ID` in Config.java to match actual CAN ID
- Test direction on real robot and set `Inverted?` preference accordingly
- Tune `kV` and `kP` on real hardware (kV ≈ 12V / max_RPS)
- Wire stall callback to LED subsystem when available
- May need to tune stall thresholds based on real load characteristics
