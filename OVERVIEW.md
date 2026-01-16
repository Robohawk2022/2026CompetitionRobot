# AGENTS.md - Context for AI Coding Assistants

This document provides essential context for AI agents working on FRC (FIRST Robotics
Competition) robot code based on this project. Read this alongside `CODING_STYLE.md` 
for complete guidance.

## Project Overview

This is an **FRC robot codebase** - software for controlling a competition robot in 
the FIRST Robotics Competition. The robot runs on a roboRIO embedded Linux computer 
and uses the WPILib framework.

### Key Domain Concepts

- **Subsystems**: Hardware mechanisms (arm, intake, shooter, drivetrain)
- **Commands**: Actions that use subsystems (move arm to position, spin shooter)
- **Periodic execution**: Code runs in a 20ms loop (50Hz), not event-driven
- **Autonomous**: 15-second period where robot runs pre-programmed routines
- **Teleop**: Driver-controlled period using Xbox controllers
- **Simulation**: WPILib supports full robot simulation without hardware

## Architecture Summary

```
frc.robot/
├── Config.java           # ALL configuration lives here (tunable via dashboard)
├── Robot.java            # Minimal - just lifecycle hooks
├── RobotContainer.java   # Wiring: creates subsystems, binds commands
├── subsystems/           # One package per mechanism
│   └── xxx/
│       ├── XxxSubsystem.java   # Business logic
│       ├── XxxHardware.java    # Interface for hardware abstraction
│       └── XxxSim.java         # Simulation implementation
├── commands/             # Complex multi-subsystem commands only
├── util/                 # Reusable utilities
└── testbots/             # Standalone test programs
```

### Core Patterns

1. **Hardware Abstraction**: Subsystems depend on interfaces, not hardware directly
2. **Centralized Config**: All tunable values in `Config.java` as `DoubleSupplier`/`BooleanSupplier`
3. **Command Factories**: Simple commands are methods in subsystems, not separate classes
4. **Simulation Support**: Where possible, subsystems have a sim implementation for testing

## Critical Rules for AI Agents

### Before Writing Any Code

2. **Check `Config.java`** - All tunable parameters must be defined there
3. **Understand the hardware interface** - Never access hardware directly from commands
3. **Understand the util package** - Don't deduplicate already-existing utility code

### When Creating a New Subsystem

Follow this exact structure:

```java
// 1. Create XxxHardware.java interface
public interface XxxHardware {
    double getSomeValue();      // Include units in javadoc
    void applyVolts(double volts);
    double getMotorAmps();
}

// 2. Create XxxSubsystem.java
public class XxxSubsystem extends SubsystemBase {
    static final boolean verboseLogging = true;

    final XxxHardware hardware;
    String currentMode;         // Always track mode for debugging

    public XxxSubsystem(XxxHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        // Set up dashboard publishing...
    }

    // Command factories go here
    public Command someCommand() { ... }
}

// 3. Create XxxSim.java implementing XxxHardware
// 4. Add config interface in Config.java
// 5. Wire up in RobotContainer.java
```

### When Adding Tunable Parameters

**Always** add to `Config.java`, never hardcode:

```java
// In Config.java
interface NewSubsystem {
    DoubleSupplier kP = pref("NewSubsystem/kP", 0.0);
    DoubleSupplier kD = pref("NewSubsystem/kD", 0.0);
    DoubleSupplier maxSpeed = pref("NewSubsystem/MaxSpeed", 10.0);
    BooleanSupplier enabled = pref("NewSubsystem/Enabled?", true);
}

// In the subsystem - use static imports
import static frc.robot.Config.NewSubsystem.*;
double p = kP.getAsDouble();
```

### When Writing Commands

**Prefer command factories in subsystems:**

```java
// Good - in XxxSubsystem.java
public Command presetCommand(Preset preset) {
    return startRun(
        () -> { currentMode = preset.name(); pid.reset(); },
        () -> { /* execute logic */ }
    );
}

// Only create separate Command classes for complex logic and multi-subsystem coordination
```

**Always in command initialization:**
- Reset PID controllers
- Set `currentMode` for debugging
- Reset timers if used

### Units Convention

**User-facing (dashboard, preferences):** feet, degrees, ft/s, deg/s

**Internal calculations:** meters, radians (WPILib standard)

```java
// Convert at boundaries
double meters = Units.feetToMeters(feet);
double radians = Units.degreesToRadians(degrees);
```

## Important Technical Details

### Motion Control Approaches

* Use P and D parameters (an I term runs the risk of integral windup)
* Use the right feedforward model (flywheel, arm, elevator, etc.)
* Reset error calculators when initializing commands
* Reset parameters (P, D, V etc.) from Config when initializing commands
* Use `TrapezoidProfile` for smooth motion when the start and end point are known

## Common Utility Methods

```java
// Voltage clamping (always do this before sending to hardware)
double volts = Util.clampVolts(calculatedVolts);

// Speed limiting
Util.clampFeetAndDegrees(speeds, maxTranslate, maxRotate);

// Angle wrapping to (-180, 180)
double wrapped = Util.degreeModulus(degrees);

// Preferences helpers
DoubleSupplier pref = Util.pref("Category/Name", defaultValue);

// Pose utilities
double distance = Util.feetBetween(pose1, pose2);
Pose2d tagPose = Util.getTagPose(tagId);
```

## Common Mistakes to Avoid

1. **Don't hardcode tunable values** - Use Config.java with preferences
2. **Don't forget voltage clamping** - Always clamp to ±12V before hardware
3. **Don't skip PID reset** - Reset in command initialize(), not constructor
4. **Don't allocate in periodic()** - Causes GC pauses, affects loop timing
5. **Don't use I term in PID** - Use PDController to avoid integral windup
6. **Don't access hardware from commands** - Go through subsystem methods
7. **Don't forget subsystem requirements** - Call `addRequirements()` in commands
8. **Don't mix units** - Convert at boundaries, be consistent internally

## Testing Approach

1. **Testbots**: Create standalone `XxxTestbot extends TimedRobot` for each subsystem
2. **Simulation**: All subsystems should work in simulation via XxxSim implementations
3. **Dashboard tuning**: Use preferences for live parameter adjustment

## FRC-Specific Context

### Competition Structure

- **Autonomous period**: 15 seconds, no driver input
- **Teleop period**: ~2 minutes, driver-controlled
- **Endgame**: Last 20 seconds, often special scoring

### Hardware & Software Ecosystem

- **roboRIO**: Main controller (ARM Linux)
- **CAN bus**: Motor controllers, sensors communicate via CAN
- **NetworkTables**: Real-time data sharing with dashboard
- **Limelight**: Vision algorithms for pose estimation and targeting
- **PathPlanner**: Popular trajectory planning tool

### Common Motor Controllers

- REV SparkMax/SparkFlex (NEO motors)
- CTRE TalonFX (Falcon motors)
- Both support onboard PID ("hardware PID")

### Dashboard Tools

- Shuffleboard (WPILib standard)
- AdvantageScope (advanced logging/replay)
- Elastic (real-time telemetry)

## File Naming Quick Reference

| Type | Pattern | Example |
|------|---------|---------|
| Subsystem | `XxxSubsystem.java` | `ArmSubsystem.java` |
| Hardware Interface | `XxxHardware.java` | `ArmHardware.java` |
| Simulation | `XxxSim.java` | `ArmSim.java` |
| Command | `XxxCommand.java` | `SwerveTeleopCommand.java` |
| Testbot | `XxxTestbot.java` | `ArmTestbot.java` |

## Quick Checklist for New Code

- [ ] Configuration in `Config.java` (not hardcoded)
- [ ] Hardware interface defined
- [ ] Simulation implementation exists
- [ ] Commands reset PID in initialize()
- [ ] Commands set `currentMode` for debugging
- [ ] Voltages clamped before hardware
- [ ] Units documented and consistent
- [ ] Dashboard telemetry added
- [ ] `Objects.requireNonNull()` for constructor params
- [ ] Subsystem requirements added to commands
