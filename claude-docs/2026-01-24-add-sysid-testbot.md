# Session: Add SysId Testbot for Swerve Drive

**Date**: 2026-01-24

## Summary

Created a testbot for System Identification (SysId) of swerve drive motors. This allows characterization of drive motors to determine accurate feedforward constants (kS, kV, kA) using WPILib's SysIdRoutine class.

## Changes Made

### Files Created
- `src/main/java/frc/robot/testbots/SysIdTestbot.java` - Standalone testbot for running SysId routines on swerve drive motors

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Added SysId characterization methods:
  - `setDriveVoltage(double volts)` - Raw voltage control for drive motor
  - `setTurnAngle(Rotation2d angle)` - Angle control for turn motor
  - `getDrivePositionMeters()` - Getter for cached drive position
  - `getDriveVelocityMps()` - Getter for cached drive velocity
  - Added `VoltageOut` request field for raw voltage control

- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Added interface methods:
  - `setDriveVoltage(double volts)` - Default no-op for simulation
  - `lockTurnMotors()` - Default no-op for simulation
  - `getAverageDrivePositionMeters()` - Default implementation using module positions
  - `getAverageDriveVelocityMps()` - Default implementation using module states

- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - Implemented SysId methods:
  - `setDriveVoltage()` - Applies voltage to all 4 drive motors
  - `lockTurnMotors()` - Points all modules forward (0 degrees)
  - `getAverageDrivePositionMeters()` - Optimized implementation using cached values
  - `getAverageDriveVelocityMps()` - Optimized implementation using cached values

- `src/main/java/frc/robot/subsystems/swerve/SwerveSim.java` - Added simulation implementations for SysId:
  - `setDriveVoltage()` - Simulates voltage-to-velocity conversion (kV = 2.0 V per m/s)
  - `lockTurnMotors()` - Points all modules forward (0 degrees)
  - Uses simulated kV constant to convert voltage to velocity for realistic simulation behavior

## SysIdTestbot Usage

### Automatic Mode (Recommended)
1. Deploy to robot: `./gradlew deploy -Probot=SysIdTestbot`
2. Enable Autonomous mode in Driver Station
3. All 4 tests run automatically in sequence (~30 seconds total)
4. Tests complete with "SysId Complete!" message

### Manual Mode (Backup)
| Button | Action |
|--------|--------|
| **A** (hold) | Quasistatic Forward |
| **B** (hold) | Quasistatic Reverse |
| **X** (hold) | Dynamic Forward |
| **Y** (hold) | Dynamic Reverse |
| **Start** | Reset encoders |

### After Running Tests
1. Export the log data from the robot
2. Load into WPILib's SysId analysis tool
3. Calculate kS, kV, kA values
4. Update `Config.java` with characterized values

## Safety Limits

The testbot automatically stops if safe limits are exceeded. Configured for **MK4i L3 modules with Kraken X60** drive motors (max theoretical velocity ~5.6 m/s):

| Limit | Value | Description |
|-------|-------|-------------|
| `MAX_DISTANCE_METERS` | 4.0m (~13ft) | Maximum distance from starting position |
| `MAX_VELOCITY_MPS` | 6.0 m/s (~20 ft/s) | Maximum drive velocity (above theoretical max) |
| `MAX_ROTATION_DEGREES` | 30° | Maximum rotation from starting heading (tight limit - wheels should stay straight) |

When a limit is exceeded:
- Motors immediately stop
- All running commands are cancelled
- Dashboard shows `SAFETY STOP` status
- Press **Start** to reset safety and continue

**Adjust limits** at the top of `SysIdTestbot.java` based on your testing area.

## Dashboard Widgets

The testbot publishes data in multiple formats for compatibility with different dashboards:

### Shuffleboard/Elastic (Sendable widgets)

| Widget | Properties |
|--------|------------|
| **Field** | 2D field visualization with robot pose |
| **SysId** | CurrentTest, AppliedVolts, PositionMeters, VelocityMps |
| **Safety** | TRIGGERED, Reason, DistFromOrigin, MaxDist, MaxVel, MaxRot |
| **Pose** | X (ft), Y (ft), Heading (deg), Gyro (deg), YawRate (dps) |
| **Modules** | FL/FR/BL/BR Velocity (mps), Angle (deg), Position (m) |
| **ButtonLog** | Last, Previous (tracks button presses) |
| **Controller** | A, B, X, Y, Start, Back, LeftBumper, RightBumper, LeftX, LeftY, RightX, RightY |

### AdvantageScope (Struct publisher)

| Topic | Type |
|-------|------|
| `SysId/Pose` | Pose2d struct (for 3D visualization) |

### Direct SmartDashboard (Elastic compatibility)

| Key | Description |
|-----|-------------|
| `Swerve/Pose X (ft)` | X position in feet |
| `Swerve/Pose Y (ft)` | Y position in feet |
| `Swerve/Pose Heading (deg)` | Heading in degrees |
| `Swerve/Gyro (deg)` | Raw gyro reading in degrees |
| `Swerve/Velocity (mps)` | Average velocity in m/s |
| `Swerve/Position (m)` | Average position in meters |
| `Swerve/Applied Volts` | Voltage being applied to motors |
| `Swerve/Test` | Current test name |
| `Swerve/Safety Triggered` | Boolean safety status |

## Button Controls

| Button | Action |
|--------|--------|
| **A** (hold) | Quasistatic Forward |
| **B** (hold) | Quasistatic Reverse |
| **X** (hold) | Dynamic Forward |
| **Y** (hold) | Dynamic Reverse |
| **Start** | Reset encoders, gyro, and odometry |
| **Back** | Zero gyro heading only |

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - robot moves when SysId tests are triggered (SwerveSim implements voltage control)
- [ ] Real hardware - not tested yet

## Known Issues / TODO

- [ ] SysId requires real hardware for useful characterization data
- [ ] After characterization, update `Config.java` with new kS, kV, kA values
- [ ] Consider adding turn motor characterization in future

## Notes for Next Session

The SysId testbot uses WPILib's `SysIdRoutine` class. Key points:
- `SysIdRoutineLog` is in `edu.wpi.first.wpilibj.sysid` (not the sysid command package)
- `Voltage` type is `edu.wpi.first.units.measure.Voltage`
- The routine logs position, velocity, and voltage data for offline analysis
- Tests should be run on a flat surface with ample space for robot movement
