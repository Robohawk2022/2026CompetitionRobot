# Session: Add CTRE Swerve Drive Subsystem

**Date**: 2025-01-17
**Duration**: ~45 minutes

## Summary

Added a complete swerve drive subsystem adapted from Robohawk2022/2025CompetitionRobot, converted from REV SparkMax to CTRE TalonFX motors with CANcoder and Pigeon2 gyro.

## Changes Made

### Files Created

| File | Description |
|------|-------------|
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` | Hardware interface for swerve drive |
| `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` | Main subsystem with odometry, pose estimation, command factories |
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` | CTRE implementation using TalonFX + CANcoder + Pigeon2 |
| `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` | Individual swerve module control (drive + turn) |
| `src/main/java/frc/robot/subsystems/swerve/SwerveSim.java` | Simulation implementation |
| `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` | Vision pose estimation (MegaTag1 and MegaTag2) |
| `src/main/java/frc/robot/util/PosePublisher.java` | Pose publishing for AdvantageScope |
| `src/main/java/frc/robot/testbots/SwerveTestbot.java` | Standalone test program |

### Files Modified

| File | Changes |
|------|---------|
| `src/main/java/frc/robot/Config.java` | Added `Swerve` interface with placeholder CAN IDs, dimensions, and tunable PID values |
| `src/main/java/frc/robot/RobotContainer.java` | Wired swerve subsystem with driver controller bindings |

## Config Changes

Added `Config.Swerve` interface with:

**Placeholder values (need to be updated for your robot):**
- `WHEEL_BASE_INCHES = 26.25` - Front-to-back wheel distance
- `TRACK_WIDTH_INCHES = 26.5` - Left-to-right wheel distance
- CAN IDs: FL=1-3, FR=4-6, BL=7-9, BR=10-12, Pigeon=13
- Angular offsets: All set to 0.0 (calibrate these!)

**Tunable values:**
- `Swerve/MaxSpeedFPS` = 15.0
- `Swerve/MaxRotationDPS` = 360.0
- `Swerve/Drive/kP` = 0.1
- `Swerve/Drive/kV` = 0.12
- `Swerve/Turn/kP` = 50.0
- `Swerve/Turn/kD` = 0.5

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `driveCommand()` | SwerveSubsystem | Field-relative driving with joystick inputs |
| `idleCommand()` | SwerveSubsystem | Sets wheels to X pattern to prevent movement |
| `zeroHeadingCommand()` | SwerveSubsystem | Zeroes the gyro |
| `resetPoseCommand()` | SwerveSubsystem | Resets pose to origin |
| `fixedSpeedCommand()` | SwerveSubsystem | Drives at fixed speeds for a duration |

## Controller Bindings (Driver)

| Button | Action |
|--------|--------|
| Left Stick | Translate (field-relative) |
| Right Stick X | Rotate |
| Start | Zero heading |
| Back | Reset pose |
| X | X-lock wheels |

## Testing Done

- [x] All files created successfully
- [ ] `./gradlew build` - **BLOCKED** by Java version (system has Java 24, needs Java 17)
- [ ] Simulation tested - Not possible until build works

## Known Issues / TODO

- [ ] **Environment issue**: Build requires Java 17 (WPILib JDK). Current system has Java 24 which is unsupported by Gradle 8.11.
- [ ] Calibrate angular offsets for each module
- [ ] Update CAN IDs to match actual robot wiring
- [ ] Measure actual chassis dimensions
- [ ] Tune drive and turn PID values on real hardware
- [ ] Add vision integration in SwerveSubsystem.periodic() using LimelightEstimator

## Architecture Notes

The swerve drive follows the codebase hardware abstraction pattern:

```
SwerveHardware (interface)
    ├── SwerveHardwareCTRE (real hardware)
    └── SwerveSim (simulation)

SwerveSubsystem
    └── Uses SwerveHardware
```

**Key differences from source (Robohawk2022):**
- Converted from REV SparkMax to CTRE TalonFX
- Converted from navX to Pigeon2
- Moved config to Config.java pattern with DoubleSupplier
- Added SwerveSim for simulation
- Followed existing codebase patterns

## Notes for Next Session

1. **To build**: Install WPILib 2026 which includes Java 17, or set JAVA_HOME to a Java 17 installation
2. Run: `./gradlew simulateJava -Probot=SwerveTestbot` to test in simulation
3. After calibrating angular offsets:
   - Point all wheels forward manually
   - Read CANcoder values from dashboard
   - Negate those values and put in `FL_ANGULAR_OFFSET`, etc.
4. Vision integration example to add to SwerveSubsystem.periodic():
   ```java
   LimelightEstimator.updateEstimateClassic("limelight", poseEstimator);
   ```
