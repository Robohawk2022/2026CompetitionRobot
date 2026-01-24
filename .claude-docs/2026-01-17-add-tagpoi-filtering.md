# Session: Add TagPOI Filtering and Update StdDev Formula

**Date**: 2026-01-17

## Summary

Added AprilTag Point of Interest (POI) filtering for the 2026 REBUILT game and updated the standard deviation formula to use the simpler ProjectBucephalus approach.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/vision/TagPOI.java` - New enum for AprilTag POI filtering

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added 2026 REBUILT AprilTag ID arrays
- `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` - Added POI filtering and new stdDev formula

## Config Changes

Added tag ID arrays organized by game element and field position:

**By Game Element:**
- `TOWER_TAG_IDS` = {1-12, 17-28} - Central scoring/climbing structures
- `OUTPOST_TAG_IDS` = {13, 14, 29, 30} - Outpost scoring stations
- `HUB_TAG_IDS` = {15, 16, 31, 32} - Alliance hub/wall area

**By Field Position:**
- `LEFT_SIDE_TAG_IDS` = {17-32} - Left half of field
- `RIGHT_SIDE_TAG_IDS` = {1-16} - Right half of field
- `CENTER_TAG_IDS` = {1-12, 17-28} - Center field elements
- `ALLIANCE_WALL_TAG_IDS` = {13-16, 29-32} - Field edge tags

**New Preference:**
- `Vision/LinearStdDevBaseline` (default: 0.08) - Baseline stdDev for ProjectBucephalus formula

## API Added

### TagPOI Enum
```java
public enum TagPOI {
    TOWER, OUTPOST, HUB,           // Game element based
    LEFT_SIDE, RIGHT_SIDE,         // Position based
    CENTER, ALLIANCE_WALL,
    ALL                            // No filtering
}
```

### LimelightEstimator Methods
```java
// Set which tags to use for pose estimation
LimelightEstimator.setActivePOI(TagPOI.TOWER);

// Get current POI
TagPOI current = LimelightEstimator.getActivePOI();
```

## StdDev Formula Change

**Old formula:** `stdDev = baseStdDev * (1 + distance/scaleFactor) / sqrt(tagCount)`

**New formula (ProjectBucephalus):** `stdDev = baseline * (avgDist^2 / tagCount)`

The new formula:
- Uses quadratic distance penalty (further = much less trust)
- Uses linear tag count benefit (more tags = more trust)
- Is simpler with fewer tuning parameters

## Testing Done

- [x] `./gradlew build` - passed

## Known Issues / TODO

- [ ] Test POI filtering in simulation with AprilTags
- [ ] Tune `linearStdDevBaseline` on real robot
- [ ] Consider auto-selecting POI based on robot position

## Notes for Next Session

The POI filtering uses `LimelightHelpers.SetFiducialIDFiltersOverride()` which persists until changed. Call `setActivePOI(TagPOI.ALL)` to disable filtering.

Dashboard telemetry shows current POI at `Vision/ActivePOI`.
