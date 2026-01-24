# Session: Add PathPlanner Integration

**Date**: 2026-01-23

## Summary

Added PathPlanner library integration for autonomous path following with swerve drive. Configured AutoBuilder, created auto chooser, and added example path/auto files.

## Changes Made

### Files Created
- `src/main/deploy/pathplanner/paths/ExamplePath.path` - Sample path going from (1,1) to (3,1) meters
- `src/main/deploy/pathplanner/autos/ExampleAuto.auto` - Sample auto routine using ExamplePath
- `src/main/deploy/pathplanner/settings.json` - PathPlanner GUI settings for the project

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `PathPlanner` configuration interface with:
  - Robot physical properties (mass, MOI, wheel COF)
  - Drive motor properties (Kraken X60 specs)
  - Path following PID tuning parameters
  - Logging enable flag

- `src/main/java/frc/robot/RobotContainer.java` - Added:
  - PathPlanner AutoBuilder configuration
  - SendableChooser for auto selection on dashboard
  - `configurePathPlanner()` method with full robot config
  - `shouldFlipPath()` for red alliance path mirroring
  - Updated `getAutonomousCommand()` to use auto chooser

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added:
  - `driveRobotRelative(ChassisSpeeds)` method for PathPlanner path following

## Config Changes

- Added `PathPlanner/Translation/kP` with default 5.0
- Added `PathPlanner/Translation/kI` with default 0.0
- Added `PathPlanner/Translation/kD` with default 0.0
- Added `PathPlanner/Rotation/kP` with default 5.0
- Added `PathPlanner/Rotation/kI` with default 0.0
- Added `PathPlanner/Rotation/kD` with default 0.0
- Added `PathPlanner/Logging?` with default true

## How to Use

### Creating Paths
1. Open PathPlanner GUI application
2. Open this project folder
3. Create paths in the Paths tab
4. Create autos in the Autos tab
5. Save - files are stored in `src/main/deploy/pathplanner/`

### Running Autonomous
1. Deploy code to robot or run simulation
2. Open Shuffleboard/SmartDashboard
3. Find "Auto Chooser" widget
4. Select desired auto routine
5. Enable autonomous mode

### Tuning Path Following
Adjust PID values in Preferences:
- `PathPlanner/Translation/kP` - Position tracking (increase if robot undershoots)
- `PathPlanner/Rotation/kP` - Heading tracking (increase if rotation undershoots)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not yet tested
- [ ] Real hardware - not tested yet

## Known Issues / TODO

- [ ] Tune translation and rotation PID gains on real robot
- [ ] Update robot physical properties (mass, MOI) from CAD
- [ ] Create actual competition auto routines
- [ ] Test path following in simulation

## Notes for Next Session

The PathPlanner vendor dependency was already installed (2026.1.2). The integration uses:
- `PPHolonomicDriveController` for swerve path following
- `RobotConfig` with module positions (Translation2d) for accurate feedforward
- Auto paths are flipped for red alliance using `shouldFlipPath()`

PathPlanner GUI can be downloaded from: https://pathplanner.dev

The `driveRobotRelative` method in SwerveSubsystem uses `MAX_WHEEL_SPEED_MPS` for desaturation, which is calculated from motor specs in Config.Swerve.
