# Session: Add Power Monitoring Subsystem

**Date**: 2026-03-14

## Summary

Created a read-only power monitoring subsystem that reads the Power Distribution Hub (PDH) to track battery voltage, total current, power draw, temperature, and identifies the top channels by current draw.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/power/PowerHardware.java` - Hardware interface with methods for channel current, total current, voltage, power, temperature, and channel count
- `src/main/java/frc/robot/subsystems/power/PowerHardwareWPILib.java` - Real hardware implementation wrapping WPILib's PowerDistribution class (REV PDH or CTRE PDP)
- `src/main/java/frc/robot/subsystems/power/PowerHardwareSim.java` - Simulation implementation returning reasonable defaults (12V, 0A, 25°C)
- `src/main/java/frc/robot/subsystems/power/PowerSubsystem.java` - Monitoring subsystem that reads all channels each cycle, finds top-drawing channels, and publishes telemetry

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `Power` config interface with `TopChannelCount` preference (default 3)
- `src/main/java/frc/robot/RobotContainer.java` - Added `PDH_CAN_ID = 1`, created PowerSubsystem with channel name map linking CAN IDs to mechanism names (Shooter, Intake, Feeder, Agitator)

## Config Changes

- Added `Power/TopChannelCount` with default 3.0

## Dashboard Telemetry

| Key | Description |
|-----|-------------|
| `Voltage` | Battery voltage |
| `CurrentTotal` | Total current draw in amps |
| `PowerTotal` | Total power in watts |
| `Temperature` | PDH temperature in Celsius |
| `TopChannel1` / `TopChannel1Amps` | Highest drawing channel name and current |
| `TopChannel2` / `TopChannel2Amps` | Second highest |
| `TopChannel3` / `TopChannel3Amps` | Third highest |
| `Channel/<name>` | Per-channel currents (verbose logging only) |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not run
- [ ] Real hardware - not tested

## Notes for Next Session

- PDH CAN ID is set to 1 — verify this matches the actual PDH on the robot
- Channel name map uses CAN IDs as channel numbers — these may not correspond to PDH channel numbers. Update the map in RobotContainer to use actual PDH channel numbers once wiring is known.
- No default command needed; periodic() handles all monitoring.
