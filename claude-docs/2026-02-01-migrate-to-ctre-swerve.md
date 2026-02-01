# Session: Migrate Custom Swerve to CTRE TunerSwerveDrivetrain

**Date**: 2026-02-01

## Summary

Migrated from custom `SwerveSubsystem` + `SwerveHardware` architecture to CTRE's `TunerSwerveDrivetrain` + `SwerveRequest` pattern.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/TunerConstants.java` - CTRE swerve drivetrain constants (motor IDs, PID gains, gear ratios, module positions)
- `src/main/java/frc/robot/subsystems/swerve/CommandSwerveDrivetrain.java` - Extends TunerSwerveDrivetrain with SysId routines, operator perspective, simulation support

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Refactored to wrap CommandSwerveDrivetrain instead of SwerveHardware interface
- `src/main/java/frc/robot/commands/swerve/SwerveTeleopCommand.java` - Updated to use `driveFieldRelative` (CTRE handles operator perspective automatically)
- `src/main/java/frc/robot/RobotContainer.java` - Changed to use `TunerConstants.createDrivetrain()`
- `src/main/java/frc/robot/testbots/SwerveTestbot.java` - Updated to use new drivetrain creation
- `src/main/java/frc/robot/testbots/VisionSimTestbot.java` - Updated to use new drivetrain creation
- `src/main/java/frc/robot/testbots/SysIdTestbot.java` - Simplified to use CTRE's built-in SysId routines

### Files Deleted (no longer needed)
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Hardware interface
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - CTRE hardware implementation
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Module abstraction

### Files Kept
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareConfig.java` - Physical constants still used by AutonomousSubsystem for PathPlanner

## Architecture Changes

### Before (Custom)
```
SwerveSubsystem
    └── SwerveHardware (interface)
            ├── SwerveHardwareCTRE (real)
            └── SwerveHardwareSim (simulation)
                    └── SwerveModule (x4)
```

### After (CTRE)
```
SwerveSubsystem (wrapper)
    └── CommandSwerveDrivetrain (CTRE)
            └── TunerSwerveDrivetrain (CTRE Phoenix 6)
```

## Key API Changes

### Driving
- `driveRobotRelative(mode, speeds)` - Uses `SwerveRequest.RobotCentric`
- `driveRobotRelative(mode, speeds, center)` - With custom center of rotation
- `driveFieldRelative(mode, speeds)` - Uses `SwerveRequest.FieldCentric` (new)

### Vision
- `submitVisionEstimate(pose, timestamp, confidence)` - Now uses CTRE's `addVisionMeasurement` internally

### Teleop
- SwerveTeleopCommand now uses `driveFieldRelative` when driverRelative mode is enabled
- CTRE's CommandSwerveDrivetrain handles operator perspective automatically via `setOperatorPerspectiveForward`

## PID Values (from TunerConstants)

| Parameter | Value |
|-----------|-------|
| Steer kP | 100 |
| Steer kD | 0.5 |
| Steer kS | 0.1 |
| Steer kV | 2.49 |
| Drive kP | 0.1 |
| Drive kS | 0 |
| Drive kV | 0.124 |
| Slip Current | 120A |
| Steer Current | 60A |

## Encoder Offsets (from TunerConstants)

| Module | Offset (rotations) |
|--------|-------------------|
| Front Left | -0.2177734375 |
| Front Right | -0.19775390625 |
| Back Left | 0.051025390625 |
| Back Right | -0.169189453125 |

**Note:** If robot wheels aren't pointing forward at startup, these offsets may need adjustment.

## Testing Done

- [x] `./gradlew build` - All tests pass
- [ ] Simulation tested - Pending
- [ ] Real hardware tested - Pending

## Known Issues / TODO

- [ ] Test in simulation to verify driving behavior
- [ ] Test on real robot to verify encoder offsets
- [ ] May need to tune PID values for real robot
- [ ] Orbit command uses custom center of rotation - verify behavior

## Notes for Next Session

1. The encoder offsets in TunerConstants are from the CTRE-Swerve reference repo and may not match your actual robot. Zero the wheels and re-measure if needed.

2. SwerveHardwareConfig.java is kept because AutonomousSubsystem uses it for PathPlanner robot configuration (mass, MOI, wheel diameter, module translations). Consider migrating these to TunerConstants in a future session.

3. CTRE's CommandSwerveDrivetrain runs odometry in a 250Hz thread (on CAN FD) which is faster than the old manual odometry. This should improve pose estimation accuracy.

4. The orbit command still works because `driveRobotRelative(mode, speeds, center)` supports custom center of rotation via `SwerveRequest.RobotCentric.withCenterOfRotation()`.
