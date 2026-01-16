# AGENTS.md

FRC robot code for Team 3373. Java 17, WPILib framework, Gradle build.

**For detailed context**: See `OVERVIEW.md` (architecture, domain concepts, patterns)
**For code style**: See `CODING_STYLE.md` (naming, comments, examples)

## Commands

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

## Project Structure

```
src/main/java/frc/robot/
├── Config.java           # ALL tunable parameters (use Preferences)
├── Robot.java            # Minimal lifecycle hooks
├── RobotContainer.java   # Wiring: creates subsystems, binds commands
├── subsystems/xxx/       # One package per mechanism
│   ├── XxxSubsystem.java # Business logic + command factories
│   ├── XxxHardware.java  # Hardware interface
│   └── XxxSim.java       # Simulation implementation
├── commands/             # Multi-subsystem commands only
├── util/                 # Reusable utilities (Util, PDController, Trapezoid)
└── testbots/             # Standalone test programs
```

## Code Style

- **Indentation**: 4 spaces
- **Naming**: PascalCase classes, camelCase methods/variables, UPPER_SNAKE constants
- **Files**: `XxxSubsystem.java`, `XxxHardware.java`, `XxxSim.java`, `XxxTestbot.java`
- **Comments**: Explain "why", document lessons learned, use `//region` markers

## Key Patterns

1. **Hardware abstraction**: Subsystems use interfaces, never access hardware directly
2. **Centralized config**: All tunables in `Config.java` as `DoubleSupplier`/`BooleanSupplier`
3. **Command factories**: Simple commands are methods in subsystems, not separate classes
4. **Units**: User-facing = feet/degrees, internal = meters/radians

## Before Writing Code

- Check `Config.java` for existing parameters
- Check `util/` package for existing helpers (`Util`, `PDController`, `Trapezoid`)
- Understand the hardware interface pattern

## Critical Rules

- Never hardcode tunable values - use `Config.java` with `Util.pref()`
- Always clamp voltages to ±12V with `Util.clampVolts()` before hardware
- Reset PID controllers in command `initialize()`, not constructor
- Don't allocate objects in `periodic()` - causes GC pauses
- Don't use I term in PID - use `PDController` to avoid integral windup
- Always call `addRequirements()` in commands
- Use `Objects.requireNonNull()` for constructor parameters

## New Subsystem Checklist

1. Create package `frc.robot.subsystems.xxx`
2. Add config interface in `Config.java`
3. Create `XxxHardware.java` interface
4. Create `XxxSubsystem.java` with command factories
5. Create `XxxSim.java` for simulation
6. Create `XxxTestbot.java` for testing
7. Wire up in `RobotContainer.java`

## Testing

- **Testbots**: Standalone `XxxTestbot extends TimedRobot` for each subsystem
- **Simulation**: All subsystems have `XxxSim` implementations
- **Dashboard**: Use Preferences for live parameter tuning

## Boundaries

**Never modify**:
- `vendordeps/` - Vendor dependency files
- `WPILib-License.md` - License file
- `.wpilib/` - WPILib configuration

**Never commit**:
- Hardcoded IP addresses or credentials
- `.gradle/` build artifacts
