# Session: Change Field-Relative to Driver-Relative Control

**Date**: 2026-01-19

## Summary

Changed the swerve drive kinematic conversion from field-relative to driver-relative, so controls are always oriented from the driver's perspective regardless of alliance color.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Changed `drive()` method at line 272 to use `Util.fromDriverRelativeSpeeds()` instead of `ChassisSpeeds.fromFieldRelativeSpeeds()`

## Implementation Details

**Before:**
```java
public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
    }
    // ...
}
```

**After:**
```java
public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
        speeds = Util.fromDriverRelativeSpeeds(speeds, getHeading());
    }
    // ...
}
```

## How Driver-Relative Works

The `Util.fromDriverRelativeSpeeds()` method (already existing in the codebase at `src/main/java/frc/robot/util/Util.java:405-436`):

1. Takes driver-relative speeds where +X = away from driver, +Y = to driver's left
2. For **blue alliance**: No change needed (driver's +X = field +X)
3. For **red alliance**: Flips X and Y (driver's +X = field -X, driver's +Y = field -Y)
4. Then converts from field-relative to robot-relative using robot heading

This ensures pushing "forward" on the joystick always moves the robot away from the driver, regardless of alliance.

## Testing Done

- [x] Code change verified correct via git diff
- [ ] `./gradlew build` - **Failed due to pre-existing errors** (not related to this change)

## Known Issues

The build fails due to pre-existing issues in `SwerveSubsystem.java`:
- The `odometryThread` field was removed but 14 references to it still exist in the file
- These errors existed before this change and need to be addressed separately

## Notes for Next Session

The pre-existing build errors need to be fixed by either:
1. Re-adding the `odometryThread` field and its initialization
2. Removing all references to `odometryThread` throughout the class

The driver-relative change itself is complete and correct.
