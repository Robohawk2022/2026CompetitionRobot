# Session: Refactor LED Subsystem for REV Blinkin

**Date**: 2026-01-25

## Summary

Refactored LED subsystem to use the REV Blinkin LED controller with signal-based API that maps robot states to LED patterns.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/led/LEDSignal.java` - Enum mapping robot states to Blinkin codes
- `src/main/java/frc/robot/subsystems/led/LEDHardwareBlinkin.java` - Blinkin hardware implementation

### Files Modified
- `src/main/java/frc/robot/subsystems/led/LEDHardware.java` - Simplified interface using signals
- `src/main/java/frc/robot/subsystems/led/LEDHardwareSim.java` - Updated for signal-based API
- `src/main/java/frc/robot/subsystems/led/LEDSubsystem.java` - Refactored to use signals
- `src/main/java/frc/robot/testbots/LEDTestbot.java` - Updated for new API
- `src/main/java/frc/robot/Config.java` - Simplified LED config

### Files Deleted
- `src/main/java/frc/robot/subsystems/led/LEDHardwareWPILib.java` - Removed (was for addressable LEDs)

## Architecture Changes

### Before (Addressable LED style)
```java
// Old approach - RGB colors and patterns
led.setSolid(LEDColor.RED);
led.setBlink(Color.kGreen);
led.setRainbow();
```

### After (Blinkin signal style)
```java
// New approach - semantic signals
led.setSignal(LEDSignal.INTAKE_FULL);     // Red strobe
led.setSignal(LEDSignal.READY_TO_SHOOT);  // Gold strobe
led.setSignal(LEDSignal.CELEBRATION);     // Rainbow party
```

## LEDSignal Enum

Maps robot states to Blinkin pattern codes:

| Signal | Blinkin Pattern | Use Case |
|--------|-----------------|----------|
| `DISABLED` | Breath Gray | Robot disabled |
| `IDLE` | Solid Blue | Enabled, nothing active |
| `INTAKING` | Solid Green | Actively collecting |
| `INTAKE_FULL` | Strobe Red | Hopper full (stall detected) |
| `EJECTING` | Solid Orange | Spitting out pieces |
| `SHOOT_RANGE_CLOSE` | Solid Green | Close to target |
| `SHOOT_RANGE_MEDIUM` | Solid Yellow | Medium shotType |
| `SHOOT_RANGE_FAR` | Solid Red | Far from target |
| `READY_TO_SHOOT` | Strobe Gold | Shooter at speed |
| `TARGET_LOCKED` | Solid Lime | Vision acquired |
| `TARGET_SEARCHING` | Breath Blue | Vision searching |
| `AUTO_MODE` | Rainbow Rainbow | Autonomous active |
| `ENDGAME_WARNING` | Strobe Blue | Last 20 seconds |
| `CLIMBING` | Color Waves Party | Climbing sequence |
| `ERROR` | Strobe Red | Fault condition |
| `ALLIANCE_RED` | Solid Red | Red alliance |
| `ALLIANCE_BLUE` | Solid Blue | Blue alliance |
| `CELEBRATION` | Rainbow Party | Scoring confirmed |
| `OFF` | Solid Black | LEDs off |

## Hardware Interface

```java
public interface LEDHardware {
    void applySignal(LEDSignal signal);
    LEDSignal getCurrentSignal();
    default void off() { applySignal(LEDSignal.OFF); }
}
```

## Config Changes

Simplified `Config.LED`:
- `PWM_PORT` = 0 (Blinkin PWM port)
- Removed `LED_COUNT`, `brightness`, `blinkPeriod`, `rainbowSpeed` (not needed for Blinkin)

## Wiring in RobotContainer

```java
// Create LED subsystem
LEDSubsystem led = new LEDSubsystem(
    Robot.isSimulation() ? new LEDHardwareSim() : new LEDHardwareBlinkin());

// Set signal based on robot state
led.setSignal(LEDSignal.IDLE);

// Wire intake stall to LED
intake.setStallCallback(stalled -> {
    if (stalled) {
        led.setSignal(LEDSignal.INTAKE_FULL);
    } else {
        led.setSignal(LEDSignal.IDLE);
    }
});

// Command-based approach
operator.a().whileTrue(led.signalWhileCommand(LEDSignal.INTAKING));
```

## Testing

Run the testbot:
```bash
./gradlew simulateJava -Probot=LEDTestbot
```

Button mappings:
- **A** - Intaking (green)
- **B** - Intake Full (red strobe)
- **X** - Ready to Shoot (gold strobe)
- **Y** - Off
- **LB (hold)** - Alliance Blue
- **RB (hold)** - Alliance Red
- **LT (hold)** - Celebration (rainbow)
- **RT (hold)** - Error (red strobe)
- **D-pad** - Target/range signals

## Blinkin Code Reference

The `LEDSignal.BlinkinCode` interface contains all REV Blinkin codes:
- `-0.99 to -0.01`: Fixed palette patterns (rainbow, fire, etc.)
- `0.01 to 0.99`: Solid colors and color patterns

See the [Blinkin User Manual](https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf) for details.

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Notes for Next Session

- Update `PWM_PORT` in Config.java if Blinkin is on different port
- Add more signals as robot features are developed
- Consider adding priority system if multiple signals fire simultaneously
- Can customize signal-to-pattern mappings by modifying LEDSignal enum
