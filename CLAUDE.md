# CLAUDE.md - AI Coding Assistant Guide

FRC robot code for Team 3373. Java 17, WPILib framework, Gradle build.

**IMPORTANT**:
- **START** by reading `claude-docs/` to check what's already been done
- **END** by logging what you did to `claude-docs/` folder

See [Session Logging](#session-logging-required) section.

## Quick Start

```bash
# Build
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Run specific testbot in simulation
./gradlew simulateJava -Probot=XxxTestbot
```

## Workflow for AI Agents

1. **Check previous sessions first** - Read `claude-docs/` to see what's been done before
2. **Read existing code** - Never propose changes to code you haven't read
3. **Check for existing utilities** - Look in `Config.java` and `util/` before creating new helpers
4. **Ask about duplicates** - If something similar exists, ask the user before creating new code
5. **Build to verify** - Run `./gradlew build` after making changes
6. **Test in simulation** - Use `./gradlew simulateJava` to verify behavior
7. **Log your work** - Document what you did in `claude-docs/` before ending session

---

## Project Overview

This is an **FRC robot codebase** - software for controlling a competition robot in the FIRST Robotics Competition. The robot runs on a roboRIO embedded Linux computer and uses the WPILib framework.

### Key Domain Concepts

- **Subsystems**: Hardware mechanisms (arm, intake, shooter, drivetrain)
- **Commands**: Actions that use subsystems (move arm to position, spin shooter)
- **Periodic execution**: Code runs in a 20ms loop (50Hz), not event-driven
- **Autonomous**: 15-second period where robot runs pre-programmed routines
- **Teleop**: Driver-controlled period using Xbox controllers
- **Simulation**: WPILib supports full robot simulation without hardware

### Competition Structure

- **Autonomous period**: 15 seconds, no driver input
- **Teleop period**: ~2 minutes, driver-controlled
- **Endgame**: Last 20 seconds, often special scoring

---

## Project Structure

```
├── claude-docs/              # AI session logs (REQUIRED - log every session here)
│   └── YYYY-MM-DD-description.md
├── src/main/java/frc/robot/
│   ├── Config.java           # ALL tunable parameters (use Preferences)
│   ├── Robot.java            # Minimal lifecycle hooks
│   ├── RobotContainer.java   # Wiring: creates subsystems, binds commands
│   ├── GameController.java   # Controller abstraction (Xbox/8BitDo/Logitech)
│   ├── subsystems/
│   │   ├── swerve/           # Swerve drive (CTRE TalonFX)
│   │   ├── limelight/        # Limelight vision (AprilTag pose estimation)
│   │   ├── questnav/         # QuestNav headset (dead-reckoning pose estimation)
│   │   ├── shooter/          # Shooter mechanism
│   │   ├── intakefront/      # Front intake with velocity control
│   │   └── led/              # LED signaling via REV Blinkin
│   ├── commands/
│   │   └── swerve/           # Standalone swerve commands (teleop, orbit)
│   ├── util/                 # Reusable utilities (Util, PDController, Trapezoid)
│   └── testbots/             # Standalone test programs
```

---

## Architecture & Core Patterns

### 1. Hardware Abstraction

Subsystems depend on interfaces, never access hardware directly. This enables simulation and testing.

```java
// Hardware interface defines what the subsystem needs
public interface ArmHardware {
    double getAngleDegrees();
    void applyVolts(double volts);
    double getMotorAmps();
}

// Subsystem uses the interface
public class ArmSubsystem extends SubsystemBase {
    final ArmHardware hardware;

    public ArmSubsystem(ArmHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
    }
}
```

### 2. Centralized Configuration

All tunable values live in `Config.java` as `DoubleSupplier`/`BooleanSupplier`:

```java
// In Config.java
interface Arm {
    DoubleSupplier kP = pref("Arm/kP", 0.1);
    DoubleSupplier kD = pref("Arm/kD", 0.01);
    DoubleSupplier maxAngle = pref("Arm/MaxAngle", 90.0);
    BooleanSupplier enabled = pref("Arm/Enabled?", true);

    // Derived values use lambdas
    DoubleSupplier halfMax = () -> maxAngle.getAsDouble() / 2.0;

    // Static constants for non-tunable values
    double GEAR_RATIO = 1.0 / 50.0;
}

// In the subsystem - use static imports
import static frc.robot.Config.Arm.*;

double p = kP.getAsDouble();
```

### 3. Command Factories

Simple commands are methods in subsystems, not separate classes:

```java
// Good - in XxxSubsystem.java
public Command presetCommand(Preset preset) {
    return startRun(
        () -> {
            currentMode = preset.name();
            pid.reset();
        },
        () -> { /* execute logic */ }
    );
}
```

**Size guideline:** Command factory methods should be ~10 lines or less. If longer:
1. Create a standalone command class in `frc.robot.commands.<subsystem>/`
2. The factory method returns `new XxxCommand(this, ...)`
3. The command class owns its state and reads config values directly

```java
// When factory would be too long, extract to a class:
public Command driveCommand(GameController controller) {
    return new SwerveTeleopCommand(this, controller);
}

// Add helper methods for commands to update subsystem state:
public void setTelemetry(String mode, boolean sniper, boolean turbo) { ... }
```

### 4. Units Convention

**User-facing (dashboard, preferences):** feet, degrees, ft/s, deg/s

**Internal calculations:** meters, radians (WPILib standard)

```java
// Convert at boundaries
double meters = Units.feetToMeters(feet);
double radians = Units.degreesToRadians(degrees);
```

---

## Code Style

### General

- **Indentation**: 4 spaces (not tabs)
- **Braces**: Opening brace on same line, closing brace on new line
- **Blank Lines**: Use to separate logical sections

### Naming Conventions

| Type | Style | Example |
|------|-------|---------|
| Classes | PascalCase | `ArmSubsystem`, `SwerveTeleopCommand` |
| Methods | camelCase | `presetCommand`, `getMotorAmps` |
| Variables | camelCase | `currentMode`, `setpoint` |
| Constants | UPPER_SNAKE | `MAX_VOLTS`, `GEAR_RATIO` |
| Packages | lowercase | `frc.robot.subsystems.arm` |

### File Naming

| Type | Pattern | Example |
|------|---------|---------|
| Subsystem | `XxxSubsystem.java` | `ArmSubsystem.java` |
| Hardware Interface | `XxxHardware.java` | `ArmHardware.java` |
| Simulation | `XxxHardwareSim.java` | `ArmHardwareSim.java` |
| Command | `XxxCommand.java` | `SwerveTeleopCommand.java` |
| Testbot | `XxxTestbot.java` | `ArmTestbot.java` |

### Variable Naming Patterns

```java
// Booleans
boolean isEnabled      // "is" prefix for state
boolean hasValue       // "has" prefix for possession
boolean atGoal()       // "at" prefix for position checks

// State variables
double currentAngle    // "current" for present state
double latestVolts     // "latest" for most recent calculated value
String currentMode     // mode tracking for debugging

// Suppliers
DoubleSupplier maxVelocity
BooleanSupplier cosineCompensation
```

### Comments & Documentation

```java
/**
 * <p>Brief description. Use complete sentences.</p>
 *
 * @param paramName description (lowercase, no period)
 * @return description (lowercase, no period)
 */

// lowercase for implementation comments
// explain "why", document lessons learned

//region Section Name ----------------------------------------------------------
// code here
//endregion
```

Common regions: `Implementation`, `Command factories`, `Constants`

### Java Language Features

```java
// Use switch expressions (Java 14+)
return switch (this) {
    case OPTION1 -> value1.getAsDouble();
    case OPTION2 -> value2.getAsDouble();
};

// Use lambda expressions
DoubleSupplier derived = () -> base.getAsDouble() * 2.0;

// Use static imports for Config
import static frc.robot.Config.Arm.maxAngle;

// Use Objects.requireNonNull() for validation
this.hardware = Objects.requireNonNull(hardware);
```

---

## Subsystem Architecture

> **Before creating a new subsystem**, check `claude-docs/` for existing implementations. A subsystem may already exist or have been started in a previous session.

### Complete Subsystem Example

```java
/**
 * Implementation of a subsystem that controls [mechanism description].
 */
public class XxxSubsystem extends SubsystemBase {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    /** Preset positions with values from configuration */
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
    final PDController pid;
    String currentMode;
    double currentValue;

    /**
     * Creates a {@link XxxSubsystem}
     * @param hardware the hardware interface (required)
     */
    public XxxSubsystem(XxxHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        this.pid = new PDController(kP, kD, tolerance);
        this.currentMode = "idle";

        // Dashboard publishing
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("ValueCurrent", () -> currentValue, null);
            if (verboseLogging) {
                builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        currentValue = hardware.getSomeValue();
    }

    private void applyVolts(double volts) {
        hardware.applyVolts(Util.clampVolts(volts));
    }

//endregion

//region Command factories -----------------------------------------------------

    /** @return a command that holds the current position */
    public Command idleCommand() {
        return run(() -> applyVolts(0));
    }

    /**
     * @param preset a preset position
     * @return a command that moves to that position
     */
    public Command presetCommand(XxxPreset preset) {
        return startRun(
            () -> {
                currentMode = preset.name();
                pid.reset();
            },
            () -> {
                double feedback = pid.calculate(currentValue, preset.value());
                applyVolts(feedback);
            });
    }

//endregion
}
```

### Hardware Interface Example

```java
/**
 * Interface for the hardware of a [subsystem] subsystem.
 */
public interface XxxHardware {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** Enables/disables the motor brake */
    void setBrake(boolean brake);

    /** @return [description] in [units] */
    double getSomeValue();

    /** @return motor output current in amps */
    double getMotorAmps();

    /**
     * Applies voltage to the motor
     * @param volts the voltage to apply (will be clamped by subsystem)
     */
    void applyVolts(double volts);
}
```

### Simulation Implementation Example

```java
/**
 * Implements {@link XxxHardware} for simulation.
 */
public class XxxHardwareSim implements XxxHardware {

    final SomeWPILibSim sim;
    final DoubleConsumer widget;

    public XxxHardwareSim() {
        this.sim = new SomeWPILibSim(
            DCMotor.getNEO(1),
            GEAR_RATIO,
            // ... parameters from Config
        );
        this.widget = createWidget();
    }

    private DoubleConsumer createWidget() {
        Mechanism2d mech = new Mechanism2d(4, 4);
        // ... configure visualization
        SmartDashboard.putData("XxxHardwareSim", mech);
        return value -> { /* update widget */ };
    }

    @Override
    public void applyVolts(double volts) {
        sim.setInputVoltage(volts);
        sim.update(Util.DT);
        widget.accept(getSomeValue());
    }

    // ... implement other interface methods
}
```

---

## Command Patterns

> **Before implementing commands**, check `claude-docs/` for established patterns in this codebase. Session logs document command approaches that worked (or didn't).

### Command Factory (Preferred)

```java
/**
 * @param preset a preset angle
 * @return a command that moves to that angle using trapezoid motion
 */
public Command presetCommand(ArmPreset preset) {
    Trapezoid trapezoid = new Trapezoid(maxVelocity, maxAcceleration);
    Timer timer = new Timer();
    return startRun(
        () -> {
            currentMode = preset.name();
            pid.reset();
            trapezoid.calculate(currentAngle, currentVelocity, preset.angle());
            timer.restart();
        },
        () -> {
            State state = trapezoid.sample(timer.get());
            closedLoop(state.position, state.velocity);
        });
}
```

### Standalone Command Class

Use standalone command classes when:
- Command logic exceeds ~10 lines (too complex for inline factory)
- Command needs its own state fields (timers, PID state, etc.)
- Command coordinates multiple subsystems

```java
/**
 * <p>Complex command with its own state.</p>
 */
public class XxxCommand extends Command {

    static final boolean enableLogging = false;

    final Subsystem1 subsystem1;
    final Subsystem2 subsystem2;

    public XxxCommand(Subsystem1 subsystem1, Subsystem2 subsystem2) {
        this.subsystem1 = Objects.requireNonNull(subsystem1);
        this.subsystem2 = Objects.requireNonNull(subsystem2);
        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize() {
        // Reset PID controllers, timers, set currentMode
    }

    @Override
    public void execute() {
        // Run every 20ms
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

---

## WPILib Command Types Reference

| Command Type | Factory Method | Purpose |
|-------------|----------------|---------|
| `InstantCommand` | `runOnce()` | Executes once, finishes immediately |
| `RunCommand` | `run()` | Runs continuously until interrupted |
| `StartEndCommand` | `startEnd()` | Action on start, cleanup on end |
| `FunctionalCommand` | `startRun()` | Full lifecycle with init + execute |
| `WaitCommand` | `Commands.waitSeconds()` | Delays for duration |
| `WaitUntilCommand` | `Commands.waitUntil()` | Waits until condition true |
| `ConditionalCommand` | `Commands.either()` | Selects between two commands |
| `SelectCommand` | `Commands.select()` | Selects from map at runtime |
| `RepeatCommand` | `.repeatedly()` | Restarts when finished |
| `DeferredCommand` | `Commands.defer()` | Creates command lazily |

```java
// Common factory examples
return runOnce(() -> hardware.resetEncoder());           // instant
return run(() -> applyVolts(0));                         // continuous
return startEnd(() -> applyVolts(6.0), () -> applyVolts(0));  // start/end
return startRun(() -> pid.reset(), () -> applyVolts(pid.calculate(current, target)));
Commands.waitSeconds(2.0);
Commands.waitUntil(() -> shooter.atSpeed());
Commands.either(arm.highCommand(), arm.lowCommand(), () -> sensor.isDetected());
```

---

## Command Lifecycle

| Method | When Called | Purpose |
|--------|-------------|---------|
| `initialize()` | Once, when scheduled | Reset PID, timers, set mode |
| `execute()` | Every 20ms | Main control logic |
| `isFinished()` | After each execute | Return `true` to end normally |
| `end(boolean)` | Once, when ending | Cleanup; `interrupted=true` if cancelled |

```java
@Override
public void initialize() {
    pid.reset();
    currentMode = "running";
    timer.restart();
    // DON'T: Create objects here (allocate in constructor)
}

@Override
public void execute() {
    applyVolts(pid.calculate(current, target));
    // DON'T: Allocate objects (GC pauses) or exceed 20ms
}

@Override
public void end(boolean interrupted) {
    applyVolts(0);
    currentMode = "idle";
}
```

---

## Command Composition Patterns

| Pattern | Factory | Decorator | End Condition |
|---------|---------|-----------|---------------|
| Sequential | `sequence(a, b, c)` | `a.andThen(b)` | Each finishes in order |
| All Parallel | `parallel(a, b)` | `a.alongWith(b)` | ALL must finish |
| Race Parallel | `race(a, b)` | `a.raceWith(b)` | ANY finishes (others stop) |
| Deadline Parallel | `deadline(d, a, b)` | `a.deadlineWith(d)` | Deadline finishes (others stop) |

```java
// Sequential
Commands.sequence(intake.deployCommand(), intake.runCommand().withTimeout(2.0));
shooter.shootCommand().andThen(shooter.stowCommand());

// Parallel variants
Commands.parallel(arm.moveCommand(), intake.runCommand());      // wait for all
Commands.race(intake.runCommand(), Commands.waitSeconds(5.0));  // first wins
Commands.deadline(drivetrain.driveCommand(), shooter.spinUpCommand()); // deadline wins

// Decorators
command.withTimeout(2.0)                   // interrupt after duration
       .until(() -> atGoal())              // interrupt when true
       .onlyWhile(() -> button.get())      // interrupt when false
       .unless(() -> skip())               // skip if true at start
       .onlyIf(() -> ready())              // skip if false at start
       .finallyDo((i) -> cleanup())        // always run at end
       .repeatedly()                       // restart when finished
```

---

## Trigger Bindings

| Method | When Starts | When Ends |
|--------|-------------|-----------|
| `onTrue()` | false→true | Command finishes |
| `onFalse()` | true→false | Command finishes |
| `whileTrue()` | Becomes true | false OR finishes |
| `whileFalse()` | Becomes false | true OR finishes |
| `toggleOnTrue()` | false→true (toggle) | Next toggle OR finishes |

```java
// Basic bindings
operator.a().whileTrue(arm.presetCommand(ArmPreset.HIGH));  // hold to run
operator.b().onTrue(shooter.shootCommand().withTimeout(1.0)); // press to start
operator.x().toggleOnTrue(intake.runCommand());              // toggle on/off

// Trigger composition
operator.a().and(arm::isAtPosition).onTrue(shooter.shootCommand());  // AND
operator.leftBumper().or(operator.rightBumper()).whileTrue(slowMode); // OR
sensor.debounce(0.1);  // 100ms debounce
```

---

## Common Autonomous Patterns

```java
// Simple sequential auto
public Command simpleAuto() {
    return Commands.sequence(
        shooter.spinUpCommand().until(() -> shooter.atSpeed()).withTimeout(2.0),
        intake.feedCommand().withTimeout(1.0),
        drivetrain.driveDistanceCommand(3.0)
    );
}

// Parallel: prepare while driving
public Command efficientAuto() {
    return Commands.deadline(
        drivetrain.driveToPositionCommand(target),
        shooter.spinUpCommand(),
        arm.scorePositionCommand()
    ).andThen(intake.feedCommand().withTimeout(0.5));
}

// Conditional auto
Commands.either(
    pickupAndScoreSequence(),
    drivetrain.parkCommand(),
    () -> vision.seesGamePiece()
);

// Safety: timeouts + cleanup
auto.withTimeout(14.5).finallyDo((i) -> { shooter.stop(); intake.stop(); });
```

---

## Wiring in RobotContainer

```java
public class RobotContainer {

    // Controllers (GameController handles Xbox/8BitDo/Logitech)
    final GameController driver = new GameController(0);
    final GameController operator = new GameController(1);

    // Subsystems
    final SwerveSubsystem swerve;
    final LimelightSubsystem limelight;

    public RobotContainer() {
        // Create hardware and subsystems
        swerve = new SwerveSubsystem(Robot.isSimulation()
                ? new SwerveHardwareSim()
                : new SwerveHardwareCTRE());

        limelight = new LimelightSubsystem(swerve);

        // Set default commands
        swerve.setDefaultCommand(swerve.driveCommand(driver));

        // Bind controls
        configureBindings();
    }

    private void configureBindings() {
        // Driver controls
        driver.leftBumper().whileTrue(swerve.orbitCommand(driver));
        driver.leftStick().onTrue(swerve.zeroPoseCommand());
        driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());
    }
}
```

---

## Dashboard Integration

### Key Naming for Alphabetical Grouping

Dashboard tools like Elastic display values in **alphabetical order**. To keep related values together, put the category first, then the qualifier:

| Pattern | Example Keys (sorted) | Why It Works |
|---------|----------------------|--------------|
| `CategoryQualifier` | `HeadingCurrent`, `HeadingDesired`, `HeadingError`, `HeadingGoal` | All heading values group together |
| `QualifierCategory` | `CurrentHeading`, `DesiredHeading`, `ErrorHeading`, `GoalHeading` | Scattered across the list |

**Standard qualifiers:**
- `Current` - where the robot actually is now
- `Desired` - where we want it to be now (moving setpoint from profile)
- `Error` - difference between desired/goal and current
- `Goal` - final destination / target

**Example:** For a command tracking both shotType and heading:
```
DistanceCurrent, DistanceDesired, DistanceError, DistanceGoal
HeadingCurrent, HeadingDesired, HeadingError, HeadingGoal
VelocityDesired, VelocityFeedback
OmegaDesired, OmegaFeedback
```

### Dashboard Code Pattern

```java
SmartDashboard.putData(getName(), builder -> {
    // Always include mode and key state
    builder.addStringProperty("Mode", () -> currentMode, null);
    builder.addBooleanProperty("AtGoal?", this::atGoal, null);

    // Category prefix groups related values together alphabetically
    builder.addDoubleProperty("AngleCurrent", () -> currentAngle, null);
    builder.addDoubleProperty("AngleDesired", () -> desiredAngle, null);
    builder.addDoubleProperty("AngleError", () -> error, null);
    builder.addDoubleProperty("AngleGoal", () -> targetAngle, null);

    // Gate verbose logging
    if (verboseLogging) {
        builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
        builder.addDoubleProperty("VoltsFeedback", () -> latestFeedback, null);
        builder.addDoubleProperty("VoltsFeedforward", () -> latestFeedforward, null);
    }
});
```

---

## Testing with Testbots

```java
/**
 * Standalone test program for the arm subsystem.
 */
public class ArmTestbot extends TimedRobot {

    ArmHardwareSim sim;
    ArmSubsystem arm;
    CommandXboxController controller;

    public ArmTestbot() {
        sim = new ArmHardwareSim();
        arm = new ArmSubsystem(sim);
        controller = new CommandXboxController(0);

        arm.setDefaultCommand(arm.idleCommand());

        controller.a().whileTrue(arm.presetCommand(ArmPreset.LOW));
        controller.b().whileTrue(arm.presetCommand(ArmPreset.HIGH));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
```

Run with: `./gradlew simulateJava -Probot=ArmTestbot`

---

## Common Utilities

> **Before creating new utilities**, check `util/` package and `claude-docs/` session logs. A helper may already exist or have been discussed in a previous session.

### Util Class

```java
// Voltage clamping (ALWAYS do this before hardware)
double volts = Util.clampVolts(calculatedVolts);

// Speed limiting
Util.clampFeetAndDegrees(speeds, maxTranslate, maxRotate);

// Angle wrapping to (-180, 180)
double wrapped = Util.degreeModulus(degrees);

// Preferences helpers
DoubleSupplier pref = Util.pref("Category/Name", defaultValue);
BooleanSupplier flag = Util.pref("Category/Flag?", false);

// Pose utilities
double shotType = Util.feetBetween(pose1, pose2);

// Alliance handling
boolean red = Util.isRedAlliance();

// Driver-relative speeds
ChassisSpeeds robotRelative = Util.fromDriverRelativeSpeeds(driverSpeeds, heading);

// Logging
Util.log("Message with %s format", arg);

// Pose publishing
Util.publishPose("KeyName", pose);
```

### Field Class

Field-related utilities including AprilTag lookups, field boundaries, and game-specific locations:

```java
// AprilTag lookups
AprilTagFieldLayout layout = Field.getFieldLayout();
AprilTag tag = Field.getTag(tagId);
Pose2d tagPose = Field.getTagPose(tagId);
Pose2d facingTag = Field.getPoseFacingTag(tagId, offsetFeet);
AprilTag closest = Field.closestTagFilteredBy(pose, tag -> tag.ID < 10);

// Field boundary checks
boolean outside = Field.isOutsideField(pose);

// Game-specific locations (2026)
Pose2d hubCenter = Field.getHubCenter();  // Alliance-aware hub center
```

### Constants

```java
Util.MAX_VOLTS        // 12.0
Util.DT               // 0.02 (20ms loop time)
Util.NAN_ROTATION     // Rotation2d with NaN
Util.NAN_POSE         // Pose2d with NaN values
Util.ZERO_SPEED       // ChassisSpeeds with 0 values
```

### PDController

Use `PDController` instead of WPILib's PIDController to avoid integral windup:

```java
final PDController pid = new PDController(kP, kD, tolerance);

// In command initialize()
pid.reset();

// In command execute()
double feedback = pid.calculate(current, target);
```

### Trapezoid

Use for smooth motion profiles:

```java
Trapezoid trapezoid = new Trapezoid(maxVelocity, maxAcceleration);
trapezoid.calculate(startPosition, startVelocity, endPosition);

// Sample the profile
State state = trapezoid.sample(elapsedTime);
double position = state.position;
double velocity = state.velocity;
```

---

## Critical Rules

### Must Do

- **Check `claude-docs/` first** - Read previous session logs before starting work
- **Check for existing code** - Search before creating new classes/utilities
- **Check before creating subsystems** - Read `claude-docs/` to see if a subsystem was started or discussed previously
- **Ask if duplicate found** - If something similar exists, ask the user before proceeding
- **Use Config.java** for all tunable values with `Util.pref()`
- **Clamp voltages** with `Util.clampVolts()` before sending to hardware
- **Reset PID** in command `initialize()`, not constructor
- **Call `addRequirements()`** in all commands
- **Use `Objects.requireNonNull()`** for constructor parameters
- **Track `currentMode`** for dashboard debugging
- **Log every session** to `claude-docs/` folder before ending

### Must Not

- **Don't hardcode** tunable values in subsystem code
- **Don't use I term** in PID - causes integral windup, use `PDController`
- **Don't allocate objects** in `periodic()` - causes GC pauses
- **Don't access hardware** directly from commands - go through subsystem
- **Don't mix units** - convert at boundaries, be consistent internally
- **Don't skip voltage clamping** - motors can be damaged

---

## Common Mistakes to Recognize

When reading existing code, watch for these issues:

1. **Hardcoded magic numbers** - Should be in Config.java
2. **Missing voltage clamp** - Check `applyVolts` calls
3. **PID reset in constructor** - Should be in `initialize()`
4. **Missing `addRequirements`** - Commands must declare subsystem dependencies
5. **Hardware access in commands** - Should go through subsystem methods
6. **I term in PID tuning** - Suggests potential windup issues

---

## Debugging Tips

### Build Failures

```bash
./gradlew build --stacktrace  # Show full error trace
./gradlew clean build         # Clean rebuild
```

### Simulation Issues

- Check that `XxxHardwareSim` implements all `XxxHardware` methods
- Verify `Util.DT` is used for simulation updates
- Check dashboard for `currentMode` to see command state

### Runtime Issues

- Enable `verboseLogging` in the subsystem
- Check `currentMode` on dashboard to verify command execution
- Watch motor amps for stalls or unexpected loads
- Verify PID parameters in Preferences/NetworkTables

---

## Session Logging (REQUIRED)

### Before Starting: Check What's Been Done

**ALWAYS read the `claude-docs/` folder before starting work.** This prevents:
- Duplicating work that was already completed
- Recreating code that was intentionally removed
- Missing context from previous sessions
- Conflicting with in-progress work

```bash
# Check recent session logs
ls -la claude-docs/

# Read the most recent logs to understand current state
```

**Before creating anything new, check for duplicates:**
1. Has this subsystem/feature been started already? Check `claude-docs/`
2. Does this utility already exist? Check `util/` package
3. Is this config parameter already defined? Check `Config.java`
4. Was this tried before and abandoned? Check session logs for context

**If you find something similar or potentially duplicate, ASK THE USER before proceeding:**

Example questions to ask:
- "I found an existing `ShooterSubsystem` in a previous session log. Should I continue that work or start fresh?"
- "There's already a `clampVolts()` method in `Util.java`. Should I use that or create a new one?"
- "I see `Arm/kP` is already defined in Config.java. Did you want me to modify it or add a new parameter?"
- "A previous session attempted this feature but noted issues with X. Should I try a different approach?"

**Never silently:**
- Overwrite existing code without asking
- Create a duplicate of something that exists
- Ignore previous session notes about failed approaches

### After Every Session: Log Your Work

**After every coding session, you MUST log what you did to the `claude-docs/` folder.**

### Log File Format

Create a new markdown file for each session: `claude-docs/YYYY-MM-DD-brief-description.md`

```markdown
# Session: [Brief Description]

**Date**: YYYY-MM-DD
**Duration**: ~X minutes

## Summary

[1-2 sentence overview of what was accomplished]

## Changes Made

### Files Created
- `path/to/NewFile.java` - [description]

### Files Modified
- `path/to/ExistingFile.java` - [what changed and why]

### Files Deleted
- `path/to/RemovedFile.java` - [reason for deletion]

## Config Changes

- Added `Category/Parameter` with default value X
- Modified `Category/OtherParam` from X to Y

## Commands Added/Modified

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `commandName()` | XxxSubsystem | [what it does] |

## Testing Done

- [ ] `./gradlew build` - passed/failed
- [ ] Simulation tested - [results]
- [ ] Specific tests run - [results]

## Known Issues / TODO

- [ ] Issue that needs follow-up
- [ ] Thing that wasn't finished

## Notes for Next Session

[Any context the next AI assistant (or human) should know]
```

### Example Log Entry

```markdown
# Session: Add Shooter Subsystem

**Date**: 2025-01-15
**Duration**: ~30 minutes

## Summary

Created the shooter subsystem with hardware abstraction, simulation, and basic commands.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java` - Main subsystem with spin-up command
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardware.java` - Hardware interface
- `src/main/java/frc/robot/subsystems/shooter/ShooterHardwareSim.java` - Flywheel simulation
- `src/main/java/frc/robot/testbots/ShooterTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added Shooter config interface
- `src/main/java/frc/robot/RobotContainer.java` - Wired up shooter subsystem

## Config Changes

- Added `Shooter/kP` with default 0.1
- Added `Shooter/kV` with default 0.02
- Added `Shooter/TargetRPM` with default 3000

## Commands Added/Modified

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `spinUpCommand()` | ShooterSubsystem | Spins flywheel to target RPM |
| `idleCommand()` | ShooterSubsystem | Stops the flywheel |

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - flywheel spins up correctly
- [ ] Real hardware - not tested yet

## Known Issues / TODO

- [ ] Need to tune kP and kV on real robot
- [ ] Add velocity feedback to dashboard

## Notes for Next Session

The shooter uses a simple feedforward + P controller. May need to add
bang-bang control near setpoint for faster spin-up.
```

### When to Log

- **End of every session** - Even if you only made small changes
- **Before switching tasks** - Log current work before starting something new
- **After completing a feature** - Document what was built
- **When encountering blockers** - Document issues for future reference

### Log File Naming

Use descriptive names that sort chronologically:
- `2025-01-15-add-shooter-subsystem.md`
- `2025-01-15-fix-arm-pid-tuning.md`
- `2025-01-16-refactor-swerve-commands.md`

---

## New Subsystem Checklist

1. [ ] Create package `frc.robot.subsystems.xxx`
2. [ ] Add config interface in `Config.java`
3. [ ] Create `XxxHardware.java` interface
4. [ ] Create `XxxSubsystem.java` with command factories
5. [ ] Create `XxxHardwareSim.java` for simulation
6. [ ] Create `XxxTestbot.java` for testing
7. [ ] Wire up in `RobotContainer.java`
8. [ ] Commands reset PID in `initialize()`
9. [ ] Commands set `currentMode` for debugging
10. [ ] Voltages clamped before hardware
11. [ ] Dashboard telemetry added
12. [ ] `Objects.requireNonNull()` for constructor params
13. [ ] **Log session to `claude-docs/`**

---

## Hardware & Software Ecosystem

### Hardware

- **roboRIO**: Main controller (ARM Linux)
- **CAN bus**: Motor controllers, sensors communicate via CAN
- **NetworkTables**: Real-time data sharing with dashboard
- **Limelight**: AprilTag-based vision for pose estimation
- **QuestNav**: Meta Quest headset for dead-reckoning pose estimation (optional)

### Motor Controllers

- REV SparkMax/SparkFlex (NEO motors)
- CTRE TalonFX (Falcon motors)
- Both support onboard PID ("hardware PID")

### Dashboard Tools

- Shuffleboard (WPILib standard)
- AdvantageScope (advanced logging/replay)
- Elastic (real-time telemetry)

---

## Boundaries

**Never modify**:
- `vendordeps/` - Vendor dependency files
- `WPILib-License.md` - License file
- `.wpilib/` - WPILib configuration

**Never delete**:
- `claude-docs/` - Session logs (only add new files, never remove old ones)

**Never commit**:
- Hardcoded IP addresses or credentials
- `.gradle/` build artifacts

**Always commit**:
- `claude-docs/*.md` - Session logs should be tracked in git

---

## Additional Resources

- **WPILib Documentation**: https://docs.wpilib.org/
- **Command-Based Programming**: https://docs.wpilib.org/en/stable/docs/software/commandbased/
- **PID Control**: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/
- **Preferences**: https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html

---

## Session Documentation Index

The `claude-docs/` folder contains session logs with implementation details. Key sessions:

### Swerve Drive
- [Add Swerve Drive CTRE](claude-docs/2025-01-17-add-swerve-drive-ctre.md) - CTRE Phoenix 6 swerve implementation
- [Fix Swerve Wheel Drift](claude-docs/2026-01-24-fix-swerve-wheel-drift.md) - Wheel drift fix
- [Refactor Swerve Commands](claude-docs/2026-01-27-refactor-swerve-commands.md) - Extract teleop/orbit to standalone command classes
- [Add SwerveToHeadingCommand](claude-docs/2026-01-27-add-swerve-to-heading-command.md) - Autonomous heading control with trapezoid profiling
- [Add SwerveToPoseCommand](claude-docs/2026-01-27-add-swerve-to-pose-command.md) - Autonomous pose control with dual trapezoid profiles

### Vision / Pose Estimation
- [Improve Limelight Pose Estimation](claude-docs/2026-01-17-improve-limelight-pose-estimation.md) - MegaTag2 integration, dynamic std devs, pose filtering
- [Add TagPOI Filtering](claude-docs/2026-01-17-add-tagpoi-filtering.md) - Point-of-interest based tag filtering
- [Odometry Improvements](claude-docs/2026-01-17-odometry-improvements.md) - Pose estimation enhancements

### Controller / Input
- [Extract GameController Class](claude-docs/2026-01-26-extract-gamecontroller-class.md) - Controller abstraction for 8BitDo, Xbox, and Logitech

### Shooter
- [Add Shooter Motors Testbot](claude-docs/2026-01-24-add-shooter-motors-testbot.md) - Testbot for shooter hardware

### Intake
- [Add IntakeFront Subsystem](claude-docs/2026-01-25-add-intake-front-subsystem.md) - Front intake with velocity control

### LED
- [Add LED Subsystem](claude-docs/2026-01-25-add-led-subsystem.md) - LED control subsystem
- [Refactor LED for Blinkin](claude-docs/2026-01-25-refactor-led-for-blinkin.md) - REV Blinkin integration

### Utilities
- [Add Command Logger](claude-docs/2026-01-17-add-command-logger.md) - Command debugging utility
- [Add Button Logging](claude-docs/2026-01-17-add-button-logging.md) - Controller input logging
- [Extract Field Utility Class](claude-docs/2026-01-27-extract-field-utility-class.md) - Field/AprilTag utilities

### Documentation
- [Add Command Documentation](claude-docs/2026-01-17-add-command-documentation.md) - Command patterns documentation
- [Update CLAUDE.md Add Logging](claude-docs/2025-01-17-update-claude-md-add-logging.md) - Session logging requirements
