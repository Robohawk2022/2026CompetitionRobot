# Session: Add LED Subsystem

**Date**: 2026-01-25

## Summary

Created a complete LED subsystem for controlling addressable LED strips (WS2812/NeoPixel) on the robot, with multiple patterns and Elastic dashboard integration for testing.

**Update**: Fixed testbot to properly respond to color/pattern changes. Added direct control methods to LEDSubsystem that can be called outside the command system. Set Main.java to run LEDTestbot by default.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/led/LEDHardware.java` - Hardware interface for LED strips
- `src/main/java/frc/robot/subsystems/led/LEDHardwareWPILib.java` - Real hardware implementation using AddressableLED
- `src/main/java/frc/robot/subsystems/led/LEDHardwareSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/led/LEDSubsystem.java` - Main subsystem with command factories
- `src/main/java/frc/robot/testbots/LEDTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added LED configuration interface

## Config Changes

Added `Config.LED` interface with:
- `PWM_PORT` = 0 (PWM port for LED strip)
- `LED_COUNT` = 60 (number of LEDs)
- `LED/Brightness` = 1.0 (0.0 to 1.0)
- `LED/BlinkPeriod` = 0.5 (seconds)
- `LED/RainbowSpeed` = 3.0 (cycle speed)

## Commands Added

| Command | Description |
|---------|-------------|
| `offCommand()` | Turns off all LEDs |
| `solidCommand(color)` | Sets solid color (LEDColor preset or Color) |
| `blinkCommand(color)` | Blinks LEDs on/off |
| `rainbowCommand()` | Animated rainbow pattern |
| `breathingCommand(color)` | Pulsing/breathing effect |
| `chaseCommand(color)` | Running light effect |
| `progressCommand(color, percent)` | Progress bar display |

## Direct Control Methods

For use outside the command system (e.g., in periodic loops):

| Method | Description |
|--------|-------------|
| `setOff()` | Turns off LEDs |
| `setSolid(color)` | Sets solid color |
| `setBlink(color)` | Blink pattern (call repeatedly) |
| `setRainbow()` | Rainbow pattern (call repeatedly) |
| `setBreathing(color)` | Breathing pattern (call repeatedly) |
| `setChase(color)` | Chase pattern (call repeatedly) |

## Preset Colors

`LEDColor` enum includes:
- OFF, RED, GREEN, BLUE, YELLOW, ORANGE
- PURPLE, CYAN, WHITE, PINK
- ALLIANCE_RED, ALLIANCE_BLUE

## Testing with Elastic

1. **Start the simulation:**
   ```bash
   ./gradlew simulateJava -Probot=LEDTestbot
   ```

2. **In Elastic, add these widgets:**
   - **LEDSubsystem**: Shows Mode, Pattern, Color name, RGB values, and Hex code
   - **LED/ColorChooser**: Dropdown to select preset colors
   - **LED/PatternChooser**: Dropdown to select patterns

3. **Controller buttons (for quick testing):**
   - A = Cycle through colors
   - B = Cycle through patterns
   - X = Breathing effect with current color
   - Y = Turn off
   - LB = Blue alliance color
   - RB = Red alliance color
   - LT = Rainbow pattern
   - RT = Green chase pattern

## Dashboard Values Published

The subsystem publishes to `LEDSubsystem/`:
- `Mode`: Current mode string (e.g., "solid", "rainbow", "blink")
- `Pattern`: Pattern enum name
- `Color`: Color name or hex code
- `ColorR`, `ColorG`, `ColorB`: RGB values (0-255)
- `ColorHex`: Hex string (e.g., "#FF0000")

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Notes for Next Session

- Hardware config uses PWM port 0 - update `Config.LED.PWM_PORT` if using a different port
- LED count is set to 60 - update `Config.LED.LED_COUNT` to match your strip
- For real robot, use `LEDHardwareWPILib` instead of simulation
- To wire up in RobotContainer:
  ```java
  led = new LEDSubsystem(
      Robot.isSimulation() ? new LEDHardwareSim() : new LEDHardwareWPILib());
  ```
