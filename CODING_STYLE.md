# Coding Style

This document provides guidelines for writing code that matches the style and 
patterns of this FRC robot project. These guidelines are derived from the 
codebase itself and reflect several years of Team 3373's experience.

## Table of Contents

- [Philosophy](#philosophy)
- [Project Structure](#project-structure)
- [Code Style](#code-style)
- [Naming Conventions](#naming-conventions)
- [Comments & Documentation](#comments--documentation)
- [Configuration Management](#configuration-management)
- [Subsystem Architecture](#subsystem-architecture)
- [Command Patterns](#command-patterns)
- [Hardware Abstraction](#hardware-abstraction)
- [Simulation](#simulation)
- [Dashboard Integration](#dashboard-integration)
- [Testing](#testing)
- [Common Utilities](#common-utilities)

---

## Philosophy

### Core Principles

1. **Favor Clarity Over Cleverness**: Code should be immediately understandable. We value explicit, verbose code over terse "clever" solutions.

2. **Copy-Paste Friendly**: This is not a vendor dependency by design. Code should be easy to copy into a current year project and modify as needed.

3. **Tunable by Design**: Most parameters should be accessible via NetworkTables Preferences so they can be tuned from the dashboard without redeploying code.

4. **Hardware Abstraction**: Always separate hardware interfaces from business logic to enable simulation and testing.

5. **Comment Liberally**: Explain the "why" not just the "what". Include lessons learned from previous years.

---

## Project Structure

### Package Organization

```
frc.robot/
├── commands/           # Command implementations (organized by subsystem)
│   ├── swerve/
│   └── vision/
├── subsystems/         # Subsystems (organized by mechanism)
│   ├── arm/
│   ├── intake/
│   ├── shooter/
│   ├── swerve/
│   └── vision/
├── testbots/          # Standalone test programs for subsystems
├── util/              # Utility classes and helpers
├── Config.java        # Centralized configuration
├── Robot.java         # Main robot class (minimal)
└── RobotContainer.java # Robot assembly and bindings
```

### Subsystem Package Structure

Each subsystem should have its own package containing:
- `XxxSubsystem.java` - Main subsystem class
- `XxxHardware.java` - Hardware interface
- `XxxSim.java` - Simulation implementation
- Supporting classes (e.g., `XxxSetpoint.java`)

---

## Code Style

### General Java Style

1. **Indentation**: 4 spaces (not tabs)
2. **Braces**: Opening brace on same line, closing brace on new line
3. **Line Length**: No hard limit, but be reasonable (~80 characters)
4. **Blank Lines**: Use to separate logical sections

### Java Language Features

```java
// Use switch expressions (Java 14+)
public double value() {
    return switch (this) {
        case OPTION1 -> value1.getAsDouble();
        case OPTION2 -> value2.getAsDouble();
    };
}

// Use lambda expressions for suppliers
DoubleSupplier derived = () -> base.getAsDouble() * 2.0;

// Use static imports for cleaner config access
import static frc.robot.Config.Arm.maxAngle;

// Don't bother with private/protected for instance variables
// Use final for fields that won't change
final ArmHardware hardware;
final PDController pid;

// Use Objects.requireNonNull() for parameter validation
this.hardware = Objects.requireNonNull(hardware);
```

### Variable Declarations

```java
// Group related variables
String currentMode;
double currentAngle;
double currentVelocity;

// Initialize at declaration when possible
final PDController pid = new PDController(p, d, tolerance);

// Use descriptive names
double latestFeedforward;
double latestFeedback;
double latestVolts;
```

---

## Naming Conventions

### Case Styles

- **Classes**: `PascalCase` (e.g., `ArmSubsystem`, `LimelightTranslateCommand`)
- **Methods**: `camelCase` (e.g., `presetCommand`, `getMotorAmps`)
- **Variables**: `camelCase` (e.g., `currentMode`, `setpoint`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `MAX_VOLTS`, `WHEEL_DIAMETER`)
- **Packages**: `lowercase` (e.g., `frc.robot.subsystems.arm`)

### Naming Patterns

**Booleans**:
```java
boolean isEnabled      // "is" prefix for state
boolean hasValue       // "has" prefix for possession
boolean atGoal()       // "at" prefix for position checks
```

**Getters**:
```java
double getAngle()      // "get" prefix for retrieving values
Rotation2d getHeading()
```

**State Variables**:
```java
double currentAngle    // "current" for present state
double latestVolts     // "latest" for most recent calculated value
String currentMode     // mode tracking for debugging
```

**Suppliers**:
```java
DoubleSupplier maxVelocity
BooleanSupplier cosineCompensation
Supplier<ChassisSpeeds> desiredSpeedSupplier
```

### File Naming

- Subsystems: `XxxSubsystem.java`
- Hardware: `XxxHardware.java`
- Simulation: `XxxSim.java` or `XxxMotorSim.java`
- Commands: `XxxCommand.java` or descriptive name like `SwerveTeleopCommand.java`
- Testbots: `XxxTestbot.java`

---

## Comments & Documentation

### Javadoc Style

```java
/**
 * <p>Brief description of the class/method. Use complete sentences.</p>
 *
 * <p>Additional paragraphs if needed to explain architecture decisions,
 * implementation notes, or lessons learned from previous years.</p>
 *
 * @param paramName description (lowercase, no period)
 * @return description (lowercase, no period)
 * @throws ExceptionType if condition
 */
```

### Implementation Comments

```java
// lowercase for implementation comments explaining logic flow
// these can be more casual and conversational

/*
 * Multi-line implementation comments also start lowercase.
 * Use these for longer explanations of complex logic.
 */
```

### Region Markers

```java
//region Section Name ----------------------------------------------------------

// code here

//endregion
```

Common regions:
- `//region Implementation`
- `//region Command factories`
- `//region Constants`
- `//region Miscellaneous math`

### Comment Guidelines

1. **Be Conversational**: Use a friendly, team-oriented tone
2. **Explain Why**: Focus on reasons and lessons learned, not just what
3. **Share Knowledge**: Include context that helps future team members
4. **Hard Learnings**: Document mistakes and their solutions

Examples from the codebase:
```java
// hard learning from previous years - your preset command
// will land you within a tolerance of your goal; if you
// just hold still at the "current angle", you might wind
// up within 2x the tolerance, which is sloppy.
```

---

## Configuration Management

### Centralized Configuration

All configuration should be defined in `Config.java` using nested interfaces:

```java
public interface Config {

    // Use nested interfaces to group related configuration
    interface SubsystemName {

        // Tunable parameters use Preferences
        DoubleSupplier parameterName = pref("Category/Parameter", defaultValue);
        BooleanSupplier featureEnabled = pref("Category/Feature?", false);

        // Derived parameters use lambdas
        DoubleSupplier derivedValue = () -> parameterName.getAsDouble() * 2.0;

        // Static constants for non-tunable values
        double GEAR_RATIO = 1.0 / 3.0;
        double WHEEL_DIAMETER = 4.0 / 12.0;
    }
}
```

### Configuration Guidelines

1. **Use DoubleSupplier/BooleanSupplier**: For runtime-tunable parameters
2. **Static Constants**: For physical properties that won't change (gear ratios, dimensions)
3. **Naming Convention**:
   - Preferences end with `?` for boolean questions: `"Swerve/CosineCompensation?"`
   - Use forward slash for hierarchy: `"Category/Subcategory/Parameter"`
4. **Derived Values**: Use lambda expressions to compute values from other suppliers
5. **Static Imports**: Use static imports in classes to keep code clean

```java
import static frc.robot.Config.Arm.p;
import static frc.robot.Config.Arm.d;
import static frc.robot.Config.Arm.maxAngle;
```

### Preference Helpers

Use `Util.pref()` helper for creating preferences:

```java
DoubleSupplier myParam = pref("Category/Parameter", 1.0);
BooleanSupplier myFlag = pref("Category/Flag?", false);
```

---

## Subsystem Architecture

### Basic Subsystem Pattern

```java
/**
 * Implementation of a subsystem that controls [mechanism description].
 */
public class XxxSubsystem extends SubsystemBase {

    /**
     * Setting this to true will make this subsystem publish low-level voltage
     * and current information
     */
    static final boolean verboseLogging = true;

    /**
     * Enum for preset positions/speeds, with values from configuration
     */
    public enum XxxPreset {
        POSITION1,
        POSITION2;

        public double value() {
            return switch (this) {
                case POSITION1 -> Config.Xxx.position1.getAsDouble();
                case POSITION2 -> Config.Xxx.position2.getAsDouble();
            };
        }
    }

//region Implementation --------------------------------------------------------

    final XxxHardware hardware;
    // ... state variables
    String currentMode;

    /**
     * Creates a {@link XxxSubsystem}
     * @param hardware the hardware interface (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public XxxSubsystem(XxxHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);
        // ... initialize state

        // Set up dashboard publishing
        SmartDashboard.putData(getName(), builder -> {
            builder.addDoubleProperty("CurrentValue", this::getCurrentValue, null);
            builder.addStringProperty("Mode", () -> currentMode, null);
            if (verboseLogging) {
                builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        // Update current state from hardware
    }

    /*
     * Private helper methods for control logic.
     * Use lowercase comments for implementation details.
     */
    private void openLoop(double volts) {
        // implementation
    }

    private void closedLoop() {
        // implementation
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that does X
     */
    public Command someCommand() {
        return startRun(
                () -> {
                    // initialization
                    currentMode = "mode-name";
                    pid.reset();
                },
                () -> {
                    // execution (runs every period)
                });
    }

//endregion

}
```

### Subsystem Guidelines

1. **Region Markers**: Use `//region Name` and `//endregion` to organize code
2. **Hardware Injection**: Accept hardware interface in constructor, use `Objects.requireNonNull()`
3. **Current Mode Tracking**: Use a `currentMode` string field to track state for debugging
4. **Command Factories**: Provide command factory methods instead of separate command classes
5. **Dashboard Publishing**: Use `SmartDashboard.putData()` with builder pattern
6. **Verbose Logging**: Include a static flag to enable/disable detailed telemetry
7. **Preset Enums**: Define preset positions/speeds as enums with values from Config

---

## Command Patterns

### Command Factory Methods

Prefer command factories in subsystems over separate command classes:

```java
/**
 * @param preset a preset angle
 * @return a command that will move the arm to that angle using a
 * trapezoid motion profile
 */
public Command presetCommand(ArmPreset preset) {
    Trapezoid trapezoid = new Trapezoid(maxVelocity, maxAcceleration);
    Timer timer = new Timer();
    return startRun(
            () -> {
                // Initialize command state
                currentMode = preset.name();
                pid.reset();
                trapezoid.calculate(
                        currentAngle,
                        currentVelocity,
                        preset.angle());
                timer.restart();
            },
            () -> {
                // Execute command (runs every period)
                State state = trapezoid.sample(timer.get());
                setpoint.set(preset.angle(), state.position, state.velocity);
                closedLoop();
            });
}
```

### Standalone Command Classes

For complex commands that coordinate multiple subsystems:

```java
/**
 * <p>Brief description of what this command does.</p>
 *
 * <p>More detailed explanation if needed, including architecture
 * decisions or implementation notes.</p>
 */
public class XxxCommand extends Command {

    /**
     * Setting this to true will make this command publish a bunch of info
     * to the dashboard that might be helpful for debugging
     */
    static final boolean enableLogging = false;

    final Subsystem subsystem;
    final Supplier<Input> inputSupplier;

    // State variables
    private double lastValue;

    /**
     * Creates a {@link XxxCommand}
     * @param subsystem the subsystem (required)
     * @param inputSupplier supplier for input (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public XxxCommand(Subsystem subsystem, Supplier<Input> inputSupplier) {

        this.subsystem = Objects.requireNonNull(subsystem);
        this.inputSupplier = Objects.requireNonNull(inputSupplier);

        addRequirements(subsystem);

        if (enableLogging) {
            SmartDashboard.putData(getClass().getSimpleName(), builder -> {
                builder.addDoubleProperty("LastValue", () -> lastValue, null);
                builder.addBooleanProperty("Running?", this::isScheduled, null);
            });
        }
    }

    @Override
    public void initialize() {
        // Setup
    }

    @Override
    public void execute() {
        // Run every period
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
```

### Command Guidelines

1. **startRun() Helper**: Use for simple commands with init and execute logic
2. **PID Reset**: Always reset PID controllers in initialization
3. **Mode Tracking**: Set `currentMode` for dashboard visibility
4. **Null Checking**: Use `Objects.requireNonNull()` for required parameters
5. **Enable Logging**: Include `enableLogging` flag for debugging
6. **Requirements**: Always call `addRequirements()` for subsystem dependencies

---

## Hardware Abstraction

### Hardware Interface Pattern

```java
/**
 * Interface for the hardware of a [subsystem] subsystem.
 */
public interface XxxHardware {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** Enables/disables the motor brake */
    void setBrake(boolean brake);

    /**
     * @return [description of what this measures] in [units]
     */
    double getSomeValue();

    /**
     * @return motor output current in amps
     */
    double getMotorAmps();

    /**
     * Applies voltage to the motor
     * @param volts the voltage to apply
     */
    void applyVolts(double volts);
}
```

### Hardware Guidelines

1. **Interface, Not Abstract Class**: Use interfaces for maximum flexibility
2. **Documentation**: Include units in return value documentation
3. **No Logic**: Interfaces should be thin wrappers over hardware
4. **Standard Methods**: Include `getMotorAmps()` and brake control where applicable
5. **Clear Units**: Be explicit about units (degrees, revolutions per second, etc.)

---

## Simulation

### Simulation Implementation

```java
/**
 * Implements the {@link XxxHardware} interface on top of a
 * [WPILib sim class] for testing in simulation.
 */
public class XxxSim implements XxxHardware {

    final SomeWPILibSim sim;
    final DoubleConsumer widget;

    public XxxSim() {
        // Configure simulation with realistic parameters
        this.sim = new SomeWPILibSim(
                DCMotor.getNEO(2),
                gearRatio,
                // ... other parameters
        );
        this.widget = createWidget();
    }

    /**
     * Creates the dashboard widget, and returns a {@link DoubleConsumer} that
     * can be used to update it.
     */
    private DoubleConsumer createWidget() {
        Mechanism2d mech = new Mechanism2d(4, 4);
        // ... configure widget
        SmartDashboard.putData("XxxSim", mech);
        return value -> {
            // update widget
        };
    }

    @Override
    public void applyVolts(double volts) {
        sim.setInputVoltage(volts);
        sim.update(Util.DT);
        widget.accept(getCurrentValue());
    }

    // ... implement other interface methods
}
```

### Simulation Guidelines

1. **Realistic Parameters**: Use real robot measurements (from Config if possible)
2. **Dashboard Widgets**: Create visual feedback using Mechanism2d
3. **Manual Control**: Add methods to manually set state for testing
4. **Update Timing**: Use `Util.DT` for simulation update intervals
5. **Logging**: Log when unsupported features are called

---


## Dashboard Integration

### SmartDashboard Publishing

```java
SmartDashboard.putData(getName(), builder -> {
    // Always include mode and key state
    builder.addStringProperty("Mode", () -> currentMode, null);
    builder.addBooleanProperty("AtGoal?", this::atGoal, null);

    // Current and target values
    // Use consistent prefixes so related values remain together when sorted by name
    builder.addDoubleProperty("ValueCurrent", () -> currentValue, null);
    builder.addDoubleProperty("ValueTarget", () -> targetValue, null);
    builder.addDoubleProperty("ValueError", () -> error, null);

    // Optional verbose logging
    if (verboseLogging) {
        builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
        builder.addDoubleProperty("VoltsFeedforward", () -> latestFeedforward, null);
        builder.addDoubleProperty("VoltsFeedback", () -> latestFeedback, null);
        builder.addDoubleProperty("VoltsTotal", () -> latestVolts, null);
    }
});
```

### Dashboard Guidelines

1. **Builder Pattern**: Use lambda in `putData()` for cleaner code
2. **Naming (1)**: Use consistent suffixes (Current, Target, Error)
3. **Naming (2)**: Use consistent prefixes so related values sort together (AngleCurrent, AngleTarget)
4. **Question Marks**: Use `?` suffix for boolean properties (e.g., "AtGoal?")
5. **Verbose Logging**: Gate detailed telemetry behind a flag
6. **Method References**: Use `this::method` instead of `() -> method()`
7. **Read-Only**: Pass `null` for setter if value shouldn't be changed from dashboard

### Pose Publishing

```java
// Use the Util helper for publishing poses
Util.publishPose("CurrentPose", currentPose);
Util.publishPose("TargetPose", targetPose);
```

---

## Testing

### Testbot Pattern

```java
/**
 * Standalone test program for the [subsystem] subsystem.
 */
public class XxxTestbot extends TimedRobot {

    XxxSim sim;
    XxxSubsystem subsystem;
    CommandXboxController controller;

    public XxxTestbot() {

        sim = new XxxSim();
        subsystem = new XxxSubsystem(sim);
        controller = new CommandXboxController(0);

        // Set default command
        subsystem.setDefaultCommand(subsystem.idleCommand());

        // Bind buttons to commands
        controller.a().whileTrue(subsystem.presetCommand(XxxPreset.PRESET1));
        controller.b().whileTrue(subsystem.presetCommand(XxxPreset.PRESET2));

        // Add tuning controls to dashboard
        SmartDashboard.putData("XxxTestbot", builder -> {
            builder.addDoubleProperty("TuningValue", () -> tuningValue, val -> tuningValue = val);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
```

### Testing Guidelines

1. **Standalone Class**: Testbots are complete `TimedRobot` implementations
2. **Simulation**: Use sim implementations of hardware
3. **Controller Bindings**: Map all key functions to controller buttons
4. **Dashboard Tuning**: Expose tuning parameters for live adjustment
5. **Minimal**: Keep testbots simple and focused on one subsystem

---

## Common Utilities

### Util Class Helpers

The `Util` class provides many helpful methods:

```java
// Voltage clamping
double volts = Util.clampVolts(calculatedVolts);
double clamped = Util.applyClamp(value, limit);

// Speed limiting
Util.clampFeetAndDegrees(speeds, maxTranslate, maxRotate);

// Angle wrapping
double wrapped = Util.degreeModulus(degrees);  // wraps to (-180, 180)

// Driver-relative speeds (handles alliance color)
ChassisSpeeds robotRelative = Util.fromDriverRelativeSpeeds(
    driverRelativeSpeeds,
    robotHeading);

// AprilTag utilities
Pose2d tagPose = Util.getTagPose(tagId);
Pose2d facingTag = Util.getPoseFacingTag(tagId, offsetFeet);
AprilTag closest = Util.closestTagFilteredBy(currentPose, tag -> test(tag));

// Pose math
double feet = Util.feetBetween(pose1, pose2);
double degrees = Util.degreesBetween(pose1, pose2);
Pose2d newPose = Util.incrementPose(oldPose, speeds);

// Alliance information
boolean red = Util.isRedAlliance();  // handles simulation correctly

// Logging
Util.log("Message with %s format", arg);

// Preferences
DoubleSupplier pref = Util.pref("Category/Name", defaultValue);
BooleanSupplier flag = Util.pref("Category/Flag?", false);

// Pose publishing
Util.publishPose("KeyName", pose);
```

### Constants in Util

```java
Util.MAX_VOLTS        // 12.0
Util.DT               // 0.02 (20ms)
Util.NAN_ROTATION     // Rotation2d with NaN
Util.NAN_POSE         // Pose2d with NaN values
Util.NAN_SPEED        // ChassisSpeeds with NaN
Util.ZERO_SPEED       // ChassisSpeeds with 0 values
Util.ZERO_STATE       // TrapezoidProfile.State with 0 values
```

---

## Units & Measurements

### Unit Consistency

**User-Facing (Preferences, Dashboard)**:
- Distance: **feet**
- Angle: **degrees**
- Angular velocity: **degrees per second**
- Linear velocity: **feet per second**

**Internal Calculations**:
- Distance: **meters**
- Angle: **radians**
- Angular velocity: **radians per second**
- Linear velocity: **meters per second**

### Unit Conversion

Always use WPILib's `Units` class:

```java
double meters = Units.feetToMeters(feet);
double feet = Units.metersToFeet(meters);
double radians = Units.degreesToRadians(degrees);
double degrees = Units.radiansToDegrees(radians);
double inchesToMeters = Units.inchesToMeters(inches);
```

### Configuration Example

```java
interface SwerveSubsystem {
    // User-facing in feet and degrees
    DoubleSupplier maxTeleopTranslate = pref("Swerve/TeleopMaxTranslate", 10.0);  // ft/s
    DoubleSupplier maxTeleopRotate = pref("Swerve/TeleopMaxRotate", 180.0);      // deg/s

    // Physical properties in convenient units
    double WHEEL_DIAM_M = Units.inchesToMeters(3.0);
    double WHEEL_CIRC_M = WHEEL_DIAM_M * Math.PI;
}
```

---

## Best Practices Summary

### Do's

- ✅ Use hardware interfaces for all hardware access
- ✅ Put all tunable parameters in Config as Preferences
- ✅ Comment liberally, especially "why" and lessons learned
- ✅ Use region markers to organize code
- ✅ Create simulation implementations for testing
- ✅ Use command factory methods in subsystems
- ✅ Reset PID controllers in command initialization
- ✅ Track current mode for debugging
- ✅ Use Objects.requireNonNull() for parameter validation
- ✅ Publish useful telemetry to SmartDashboard
- ✅ Use static imports for Config values
- ✅ Include units in variable names or comments
- ✅ Create testbots for isolated subsystem testing

### Don'ts

- ❌ Don't hardcode tunable values in subsystem code
- ❌ Don't access hardware directly from commands
- ❌ Don't use I term in PID (integral windup issues)
- ❌ Don't forget to clamp voltages to ±12V
- ❌ Don't mix units (feet with meters, degrees with radians)
- ❌ Don't create separate command classes for simple commands
- ❌ Don't forget to add subsystem requirements to commands
- ❌ Don't allocate objects in periodic() (performance)
- ❌ Don't use clever abbreviated names (be explicit)
- ❌ Don't skip comments on non-obvious logic

---

## Quick Reference

### Creating a New Subsystem

1. Create package: `frc.robot.subsystems.xxx`
2. Add config interface in `Config.java`
3. Create `XxxHardware.java` interface
4. Create `XxxSubsystem.java` extending `SubsystemBase`
5. Create `XxxSim.java` implementing `XxxHardware`
6. Create `XxxTestbot.java` for testing
7. Add to `RobotContainer.java`

### Creating a New Command

1. Add command factory method to subsystem, OR
2. Create `XxxCommand.java` extending `Command`
3. Inject dependencies via constructor
4. Use `Objects.requireNonNull()` for validation
5. Add subsystem requirements
6. Implement initialize/execute/end/isFinished

### Adding a Tunable Parameter

1. Add to appropriate interface in `Config.java`:
   ```java
   DoubleSupplier paramName = pref("Category/Name", defaultValue);
   ```
2. Static import in subsystem:
   ```java
   import static frc.robot.Config.Subsystem.paramName;
   ```
3. Use in code:
   ```java
   double value = paramName.getAsDouble();
   ```

---

## Additional Resources

- **WPILib Documentation**: https://docs.wpilib.org/
- **Command-Based Programming**: https://docs.wpilib.org/en/stable/docs/software/commandbased/
- **PID Control**: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/
- **Preferences**: https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html

---

*This style guide is a living document. As the codebase evolves and we learn from new experiences, these guidelines should be updated to reflect our best practices.*

