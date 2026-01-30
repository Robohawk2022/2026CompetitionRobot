# Claude's Changes - Session Log Archive

This document contains all Claude AI coding session logs merged into a single file for easy reference.

---

# 2025-01-17: Update CLAUDE.md and Add Session Logging

**Duration**: ~20 minutes

## Summary

Merged OVERVIEW.md and CODING_STYLE.md into a comprehensive CLAUDE.md file, then added session logging requirements and "check before creating" guidance throughout the document.

## Changes Made

### Files Created
- `claude-docs/README.md` - Explains the purpose of the claude-docs folder

### Files Modified
- `CLAUDE.md` - Complete rewrite merging three files plus adding logging requirements:
  - Fixed header (was incorrectly labeled "AGENTS.md")
  - Added "Workflow for AI Agents" section with 7 steps
  - Added "Wiring in RobotContainer" example (was missing)
  - Added "Common Mistakes to Recognize" section
  - Added "Debugging Tips" section
  - Added comprehensive "Session Logging (REQUIRED)" section

## Notes for Next Session

The CLAUDE.md file is now comprehensive (~870 lines) and includes:
- All architecture and pattern guidance
- Complete code examples
- Session logging requirements

---

# 2025-01-17: Add CTRE Swerve Drive Subsystem

**Duration**: ~45 minutes

## Summary

Added a complete swerve drive subsystem adapted from Robohawk2022/2025CompetitionRobot, converted from REV SparkMax to CTRE TalonFX motors with CANcoder and Pigeon2 gyro.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Hardware interface for swerve drive
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Main subsystem with odometry, pose estimation, command factories
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - CTRE implementation using TalonFX + CANcoder + Pigeon2
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Individual swerve module control (drive + turn)
- `src/main/java/frc/robot/subsystems/swerve/SwerveSim.java` - Simulation implementation
- `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` - Vision pose estimation (MegaTag1 and MegaTag2)
- `src/main/java/frc/robot/util/PosePublisher.java` - Pose publishing for AdvantageScope
- `src/main/java/frc/robot/testbots/SwerveTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `Swerve` interface with placeholder CAN IDs, dimensions, and tunable PID values
- `src/main/java/frc/robot/RobotContainer.java` - Wired swerve subsystem with driver controller bindings

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `driveCommand()` | SwerveSubsystem | Field-relative driving with joystick inputs |
| `idleCommand()` | SwerveSubsystem | Sets wheels to X pattern to prevent movement |
| `zeroHeadingCommand()` | SwerveSubsystem | Zeroes the gyro |
| `resetPoseCommand()` | SwerveSubsystem | Resets pose to origin |
| `fixedSpeedCommand()` | SwerveSubsystem | Drives at fixed speeds for a duration |

---

# 2026-01-17: Improve Limelight AprilTag Pose Estimation

## Summary

Improved the Limelight vision pose estimation for better robot localization. Integrated vision into the swerve drive pose estimator with dynamic standard deviations, field boundary validation, and pose jump detection.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/vision/VisionEstimateResult.java` - Result class for vision processing

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Added `getYawRateDps()` method
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - Implemented `getYawRateDps()`
- `src/main/java/frc/robot/subsystems/swerve/SwerveSim.java` - Implemented `getYawRateDps()`
- `src/main/java/frc/robot/Config.java` - Expanded `Limelight` interface with tunable vision parameters
- `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` - Complete refactor with new features
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Integrated vision into `periodic()`

## LimelightEstimator Improvements

1. **Fixed MegaTag2 bug** - Now uses correct API
2. **Dynamic standard deviation** - Formula: `stdDev = baseStdDev * (1 + shotType/scaleFactor) / sqrt(tagCount)`
3. **Field boundary validation** - Rejects poses outside field
4. **Pose jump detection** - Rejects if >2m from current estimate
5. **Configurable thresholds** - All filtering parameters read from Config

---

# 2026-01-17: Odometry Improvements - High-Frequency Odometry & Diagnostics

**Duration**: ~30 minutes

## Summary

Implemented high-frequency odometry (250Hz) using CTRE synchronized reads with latency compensation, plus comprehensive dashboard diagnostics for tracking drift, vision rejection stats, and odometry health.

## Changes Made

### Files Created
- `src/main/java/frc/robot/util/TimestampedPose.java` - Record for pose with timestamp
- `src/main/java/frc/robot/util/CircularBuffer.java` - Generic circular buffer for pose history
- `src/main/java/frc/robot/subsystems/swerve/OdometryThread.java` - High-frequency (250Hz) odometry thread
- `src/main/java/frc/robot/subsystems/swerve/OdometryDiagnostics.java` - Tracks and publishes odometry health metrics

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Added StatusSignal fields with 250Hz update frequency
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Added interface methods for HF odometry
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - Implemented signal collection
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Integrated OdometryThread and diagnostics
- `src/main/java/frc/robot/Config.java` - Added `Odometry` config interface

---

# 2026-01-17: Add FRC Command Documentation to CLAUDE.md

## Summary

Added concise WPILib command documentation to CLAUDE.md covering command types, lifecycle, composition patterns, trigger bindings, and autonomous patterns.

## New Documentation Sections

1. **WPILib Command Types Reference** - Table of all built-in command types
2. **Command Lifecycle** - Diagram and method explanations
3. **Command Composition Patterns** - Sequential, parallel, race, deadline
4. **Trigger Bindings** - All binding methods and composition
5. **Common Autonomous Patterns** - Example auto routines

---

# 2026-01-17: Add CommandLogger Utility Class

## Summary

Created a CommandLogger utility class that hooks into WPILib's CommandScheduler to log command lifecycle events (initialize, finish, interrupt) with SmartDashboard telemetry.

## Changes Made

### Files Created
- `src/main/java/frc/robot/util/CommandLogger.java` - Utility class for command logging

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added logging preferences
- `src/main/java/frc/robot/RobotContainer.java` - Added `CommandLogger.enable()` call

## Features

- **Event Logging**: Logs INIT, FINISH, and INTERRUPT events
- **Duration Tracking**: Reports how long each command ran
- **Execute Logging**: Optional rate-limited logging of active commands
- **SmartDashboard Telemetry**: ActiveCount, ActiveCommands, TotalRun, Enabled

---

# 2026-01-17: Add Button Logging to CommandLogger

## Summary

Extended CommandLogger to log all Xbox controller button presses, including joystick buttons (L/R stick press), D-pad, bumpers, triggers, and face buttons.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/util/CommandLogger.java` - Added button logging functionality
- `src/main/java/frc/robot/Robot.java` - Added `CommandLogger.pollButtons()` call
- `src/main/java/frc/robot/RobotContainer.java` - Added controller declarations and registration

## Buttons Logged

All Xbox buttons: A, B, X, Y, LBumper, RBumper, Back, Start, LStick, RStick, D-pad, LTrigger, RTrigger

---

# 2026-01-17: Add TagPOI Filtering and Update StdDev Formula

## Summary

Added AprilTag Point of Interest (POI) filtering for the 2026 REBUILT game and updated the standard deviation formula to use the simpler ProjectBucephalus approach.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/vision/TagPOI.java` - New enum for AprilTag POI filtering

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added 2026 REBUILT AprilTag ID arrays
- `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` - Added POI filtering and new stdDev formula

## StdDev Formula Change

**Old formula:** `stdDev = baseStdDev * (1 + shotType/scaleFactor) / sqrt(tagCount)`
**New formula (ProjectBucephalus):** `stdDev = baseline * (avgDist^2 / tagCount)`

---

# 2026-01-18: Add Limelight Simulation

## Summary

Implemented a realistic Limelight simulation (`LimelightSim`) that simulates AprilTag detection based on robot position, camera FOV, and tag visibility.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/vision/LimelightSim.java` - Core simulation class
- `src/main/java/frc/robot/testbots/VisionSimTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `LimelightSim` config interface
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Integrated LimelightSim

## LimelightSim Features

1. **Visibility Calculation**: Distance bounds, FOV check, tag orientation
2. **Realistic Metrics**: tx/ty angles, tag area, ambiguity estimation
3. **Noise Simulation**: Gaussian noise on position/rotation, random frame drops
4. **NetworkTables Publishing**: Publishes botpose_wpiblue, botpose_orb_wpiblue, tv, tx, ty, ta, tid

---

# 2026-01-19: Add Turbo and Sniper Speed Modes

## Summary

Added turbo and sniper speed modes to the swerve drive, controlled by the driver's trigger axes.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added sniperFactor, turboFactor, applySniperToRotation
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added new `driveCommand()` overload
- `src/main/java/frc/robot/RobotContainer.java` - Added swerve subsystem with turbo/sniper

## Control Mapping

| Control | Action |
|---------|--------|
| Left Stick | Translation (forward/strafe) |
| Right Stick X | Rotation |
| Left Trigger (>50%) | Sniper mode - slow, precise (25%) |
| Right Trigger (>50%) | Turbo mode - fast (150%) |

---

# 2026-01-19: Cache CTRE Encoder Values in Periodic

## Summary

Implemented caching for CTRE Phoenix 6 status signals to reduce CAN bus traffic. All encoder and gyro values are now batch-refreshed once per cycle using `BaseStatusSignal.refreshAll()`.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Added cached value fields, `refreshSignals()` method
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - Implemented batch refresh
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Added `refreshSignals()` default method
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added refresh call in `periodic()`

## Technical Details

Before: ~16+ CAN reads per 50Hz cycle when called from periodic()
After: All 13 signals batch-refreshed in a single CAN transaction

---

# 2026-01-19: Add SwerveTeleopSpeedSupplier

## Summary

Created a `SwerveTeleopSpeedSupplier` class to encapsulate teleop joystick input handling and simplified the RobotContainer controller setup.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Joystick input handling class

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `deadzone` config parameter
- `src/main/java/frc/robot/RobotContainer.java` - Updated to use SwerveTeleopSpeedSupplier

---

# 2026-01-19: Change Field-Relative to Driver-Relative Control

## Summary

Changed the swerve drive kinematic conversion from field-relative to driver-relative, so controls are always oriented from the driver's perspective regardless of alliance color.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Changed `drive()` to use `Util.fromDriverRelativeSpeeds()`

## How Driver-Relative Works

1. Takes driver-relative speeds where +X = away from driver, +Y = to driver's left
2. For **blue alliance**: No change needed
3. For **red alliance**: Flips X and Y
4. Then converts from field-relative to robot-relative using robot heading

---

# 2026-01-19: Add Swerve Cosine Compensation and Angle Motor Optimization

## Summary

Added cosine compensation to swerve module kinematics and implemented an optimization to skip angle motor movement when the drive motor is stationary.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added CosineCompensation and MinSpeedForAngle preferences
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Rewrote `setDesiredState()` method
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` - Added `getAllSignals()` default method
- `src/main/java/frc/robot/subsystems/swerve/OdometryDiagnostics.java` - Added overloaded `update()` method

## Technical Details

**Cosine Compensation**: Scales drive velocity by `cos(angleError)` so the wheel delivers correct effective speed.
**Angle Motor Skip**: When robot is stationary, doesn't command turn motor to prevent jitter.

---

# 2026-01-19: Remove High-Frequency Odometry Thread

## Summary

Removed the disabled high-frequency `OdometryThread` implementation and simplified to periodic-only (50Hz) odometry updates.

## Changes Made

### Files Deleted
- `src/main/java/frc/robot/subsystems/swerve/OdometryThread.java`
- `src/main/java/frc/robot/util/TimestampedPose.java`

### Files Modified
- `SwerveSubsystem.java` - Removed odometryThread field and HF methods
- `SwerveHardware.java` - Removed HF odometry interface methods
- `SwerveHardwareCTRE.java` - Removed `getAllSignals()` override
- `SwerveModule.java` - Removed signal getter methods
- `OdometryDiagnostics.java` - Removed frequency tracking
- `Config.java` - Removed HF odometry config

---

# 2026-01-19: Add Swerve Module Rotation Logging

## Summary

Added motor rotation logging to Shuffleboard for debugging swerve modules.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Added CAN ID fields and verbose logging

## Dashboard Output (when Logging/Swerve? enabled)

- DriveRotations, TurnRotations
- DriveCANID, TurnCANID, EncoderCANID

---

# 2026-01-19: Fix Controller Input Axis Mapping for 8BitDo

## Summary

Fixed controller axis mapping for 8BitDo controller. Changed from Xbox controller named methods to raw axis access with explicit axis indices.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Added axis constants, trigger normalization
- `src/main/java/frc/robot/testbots/SwerveTestbot.java` - Updated to use raw axis access

## 8BitDo Controller Axis Layout

- Axis 0: Left stick X
- Axis 1: Left stick Y
- Axis 2: Right stick X
- Axis 3: Right stick Y
- Axis 4: Left trigger (-1 to 1)
- Axis 5: Right trigger (-1 to 1)

---

# 2026-01-19: Fix Turbo Mode in Simulation

## Summary

Fixed turbo mode not working in simulation. Two issues: controller axis mapping and turbo speed being negated by desaturation.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `UseXboxMapping?` toggle
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Added separate axis constants for 8BitDo and Xbox
- `src/main/java/frc/robot/testbots/SwerveTestbot.java` - Replaced hardcoded mapping with SwerveTeleopSpeedSupplier
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added `drive()` overload with custom max speed

---

# 2026-01-23: Refactor Swerve Config to Reduce Preference Overuse

## Summary

Moved hardware-specific swerve constants out of Preferences into a new `SwerveHardwareConfig` class. These values are only read at startup so Preferences provided no benefit.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareConfig.java` - Hardware constants

### Files Modified
- `src/main/java/frc/robot/Config.java` - Removed CAN IDs, angular offsets, PID constants
- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` - Changed import
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` - Updated PID configuration

---

# 2026-01-23: Rename Simulation Classes to XxxHardwareSim

## Summary

Updated the naming convention for hardware simulation classes from `XxxSim` to `XxxHardwareSim` for clarity.

## Changes Made

### Files Renamed
- `SwerveSim.java` -> `SwerveHardwareSim.java`
- `LimelightSim.java` -> `LimelightHardwareSim.java`

### Files Modified
- `CLAUDE.md` - Updated naming convention documentation
- `RobotContainer.java`, `SwerveTestbot.java`, `VisionSimTestbot.java`, `SwerveHardware.java` - Updated imports

---

# 2026-01-23: Add Claude Code Hooks and MCP Servers for Documentation

## Summary

Created Claude Code hooks to remind Claude to check WPILib docs and Chief Delphi before implementing FRC code. Added MCP server for FRC documentation lookup.

## Changes Made

### Files Created
- `.claude/hooks/check-docs-reminder.sh` - UserPromptSubmit hook
- `.claude/hooks/wpilib-docs.sh` - Helper script for doc URLs
- `.claude/hooks/chief-delphi.sh` - Helper script for Chief Delphi search
- `mcp-server/frc_docs_server.py` - MCP server for documentation

### Files Modified
- `.claude/settings.local.json` - Added hook configuration
- `.mcp.json` - Added frc-docs MCP server configuration

## MCP Tools (frc-docs)

- `wpilib_docs` / `wpilib_docs_url` - WPILib documentation
- `chief_delphi_search` - Generate Chief Delphi search URL
- `ctre_docs` / `ctre_docs_url` - CTRE Phoenix 6 documentation
- `list_doc_topics` - List all available topics
- `frc_resources` - Get URLs for common FRC resources

---

# 2026-01-23: Add FRC NetworkTables MCP Server

## Summary

Created an MCP server that connects to the robot's NetworkTables, allowing Claude to query live robot state during simulation.

## Changes Made

### Files Created
- `mcp-server/frc_networktables_server.py` - Python MCP server
- `mcp-server/requirements.txt` - Python dependencies
- `mcp-server/setup.sh` - Setup script
- `mcp-server/README.md` - Documentation
- `.mcp.json` - MCP server configuration

## MCP Tools (frc-networktables)

- `get_robot_state` - Enabled/disabled, mode, alliance
- `get_robot_pose` - Position and heading
- `get_swerve_status` - Velocities, module states
- `get_vision_status` - Tag detection, pose estimates
- `get_active_commands` - Running commands
- `get_odometry_diagnostics` - Drift, vision accept rate
- `get_subsystem_status` - Query any subsystem
- `get_preference` / `list_preferences` - Config values
- `get_networktables_value` / `list_networktables` - Raw NT access
- `connection_status` - Check NT connection

---

# 2026-01-23: Add Simulation Input Control to MCP Server

## Summary

Added 4 new tools to the `frc-networktables` MCP server to control robot simulation inputs via the HALSim WebSocket protocol.

## Changes Made

### Files Modified
- `build.gradle` - Added WebSocket server configuration
- `.mcp-server/requirements.txt` - Added websockets dependency
- `.mcp-server/frc_networktables_server.py` - Added simulation control tools

## New MCP Tools

- `simulation_control_status` - Check if HALSim WebSocket control is available
- `set_robot_mode` - Enable/disable robot and set mode
- `set_joystick_input` - Set Xbox controller axes, buttons, and D-pad
- `set_alliance` - Set alliance color and station

---

# 2026-01-23: Add Orbit Drive Command for Reef Scoring

## Summary

Implemented an "orbit drive" mode that allows the robot to orbit around the 2026 Reef scoring structure while automatically facing it.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `Orbit` interface with configuration
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added `orbitCommand()`
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Added convenience method
- `src/main/java/frc/robot/RobotContainer.java` - Added left bumper binding for orbit mode

## Controls

- **Left Bumper (hold)**: Activate orbit mode
- **Left Stick Y**: Move toward/away from Reef
- **Left Stick X**: Orbit around Reef
- **Right Stick X**: Fine-tune heading (30% trim authority)
- **Triggers**: Turbo/sniper speed modifiers still work

---

# 2026-01-23: Add PathPlanner Integration

## Summary

Added PathPlanner library integration for autonomous path following with swerve drive.

## Changes Made

### Files Created
- `src/main/deploy/pathplanner/paths/ExamplePath.path` - Sample path
- `src/main/deploy/pathplanner/autos/ExampleAuto.auto` - Sample auto routine
- `src/main/deploy/pathplanner/settings.json` - PathPlanner GUI settings

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `PathPlanner` configuration
- `src/main/java/frc/robot/RobotContainer.java` - Added AutoBuilder configuration
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added `driveRobotRelative()`

---

# 2026-01-23: Fix 8BitDo Ultimate DirectInput Button Mapping

## Summary

Fixed button mapping for 8BitDo Ultimate controller in DirectInput (D) mode.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Updated button and axis constants

## Button Mapping Changes (WPILib 1-indexed)

| Button | Old Value | New Value |
|--------|-----------|-----------|
| X | 3 | 4 |
| Y | 4 | 5 |
| Left Bumper | 5 | 7 |
| Right Bumper | 6 | 8 |
| Back | 7 | 11 |
| Start | 8 | 12 |
| Left Stick Click | 9 | 14 |
| Right Stick Click | 10 | 15 |

---

# 2026-01-23: Fix HALSim WebSocket Simulation Control

## Summary

Fixed the MCP server's HALSim WebSocket control by adding the critical `">new_data": true` flag to DriverStation messages.

## Changes Made

### Files Modified
- `.mcp-server/frc_networktables_server.py` - Added new_data signaling to WebSocket messages

## Root Cause

The MCP server was sending joystick data but was NOT sending the `">new_data"` flag, so the robot code never saw the updated values.

## Solution

1. Added `">new_data": True` to `set_driver_station_mode()`
2. Added `signal_new_data()` helper function
3. Modified `set_joystick_state()` to call `signal_new_data()` after sending data
4. Updated `drive_robot()` to send updates at 50Hz

---

# 2026-01-23: Orbit Command Rotation Priority

## Summary

Implemented rotation priority for the orbit command to prevent translation from starving heading control when rotation demands are high.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `rotationPriority` parameter
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added rotation priority logic

## How It Works

With default `rotationPriority = 0.5`:
- If rotation uses 50% or less of max omega: no scaling
- If rotation uses 70% of max omega: translation scaled to 80%
- If rotation uses 90% of max omega: translation scaled to 60%
- Translation never goes below 30%

---

# 2026-01-23: Orbit Turbo Mode Fix

## Summary

Modified orbit turbo mode to use 90% of the swerve drive max speed instead of applying the turboFactor multiplier.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `TurboSpeedFraction` and `RotationTrimAuthority`
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Modified turbo handling

## Behavior Change

**Before**: Orbit turbo applied 1.5x to orbit speeds
**After**: Orbit turbo uses 90% of swerve max speed

---

# 2026-01-24: Fix SwerveSubsystem Build Errors

## Summary

Fixed 2 compilation errors in SwerveSubsystem.java caused by missing `drivePreserveRotation` method, cleaned up dead code, and fixed deprecation warnings.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Added `drivePreserveRotation(double vx, double vy, double omega, double maxSpeed)` method
  - Drives with rotation-priority desaturation for heading control

- `src/main/java/frc/robot/subsystems/vision/LimelightHardwareSim.java`
  - Removed unused class field and duplicate variable

- `src/main/java/frc/robot/testbots/SysIdTestbot.java`
  - Fixed deprecated method calls

## drivePreserveRotation Algorithm

1. Convert field-relative speeds to robot-relative
2. Calculate module states and find max wheel speed
3. If within limits, drive normally
4. If over limit:
   - Calculate how much rotation alone would use
   - If rotation alone exceeds max, scale rotation and skip translation
   - Otherwise, scale translation to fit in remaining headroom
5. Apply standard desaturation as safety net

---

# 2026-01-24: Point Swerve Wheels Forward on Startup

## Summary

Added initialization code to point all swerve wheels forward (0 degrees) when the robot starts up.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java`
  - Added `lockTurnMotors()` call at the end of the constructor
  - This commands all four modules to face forward immediately when hardware initializes

- `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareSim.java`
  - Added constructor with `lockTurnMotors()` call for consistency with real hardware

## Technical Details

The `lockTurnMotors()` method already existed (for SysId characterization) and sets all turn motors to 0 degrees. By calling it in the hardware constructors, the wheels will be commanded forward as soon as the swerve hardware is created during robot initialization.

## Testing Done

- [x] `./gradlew build` - passed
