# Session: Extract Field Utility Class

**Date**: 2026-01-27

## Summary

Created a new `Field` utility class to centralize field-related operations including AprilTag lookups, field boundary checks, and game-specific locations. Moved all AprilTag-related methods from `Util` to `Field` and added the hub center calculation from `SwerveOrbitHubCommand`.

## Changes Made

### Files Created
- `src/main/java/frc/robot/util/Field.java` - New utility class for field operations

### Files Modified
- `src/main/java/frc/robot/util/Util.java`
  - Removed AprilTag-related methods (moved to Field)
  - Removed unused imports (AprilTag, AprilTagFieldLayout, AprilTagFields, Predicate)

- `src/main/java/frc/robot/commands/swerve/SwerveOrbitHubCommand.java`
  - Removed `getHubLocation()` method (now in Field.getHubCenter())
  - Updated to use `Field.getHubCenter()`
  - Removed unused Rotation2d import

- `src/main/java/frc/robot/subsystems/limelight/LimelightSim.java`
  - Updated to use `Field.getFieldLayout()` instead of `Util.getFieldLayout()`

- `src/main/java/frc/robot/subsystems/limelight/LimelightTarget.java`
  - Updated to use `Field.getTagPose()` instead of `Util.getTagPose()`

- `src/main/java/frc/robot/subsystems/limelight/LimelightSubsystem.java`
  - Updated to use `Field.isOutsideField()` and `Field.getTagPose()`

- `src/main/java/frc/robot/subsystems/questnav/QuestNavSubsystem.java`
  - Updated to use `Field.isOutsideField()`

- `CLAUDE.md`
  - Updated Common Utilities section to document Field class

## Field Class API

### Field Layout
- `setFieldLayout(AprilTagFields)` - Set which field layout to use (call early in Main)
- `getFieldLayout()` - Get the current AprilTagFieldLayout

### AprilTag Lookups
- `getTag(int id)` - Get an AprilTag by ID
- `getTagPose(int id)` - Get a tag's Pose2d by ID
- `getPoseFacingTag(int tagId, double offsetFeet)` - Get a pose facing a tag at offset
- `closestTagFilteredBy(Pose2d, Predicate<AprilTag>)` - Find closest tag matching criteria

### Field Boundaries
- `isOutsideField(Pose2d)` - Check if a pose is outside field boundaries

### Game-Specific Locations (2026)
- `getHubCenter()` - Get alliance-aware hub center position

## Architecture Notes

The `Field` class follows the same lazy initialization pattern as the original code in `Util`:
- Field layout is loaded on first access
- Tag maps are precomputed once and cached
- Game-specific locations (like hub center) use `Util.isRedAlliance()` for alliance awareness

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested
- [ ] Real hardware - not tested

## Known Issues / TODO

- None identified

## Notes for Next Session

The separation of concerns is now:
- `Util` - General robot utilities (math, logging, preferences, alliance detection)
- `Field` - Field-specific utilities (AprilTags, boundaries, game locations)

The `Field.getHubCenter()` method is specific to the 2026 game. For future seasons, update or remove this method as appropriate.
