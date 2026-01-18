# Session: Add Limelight Simulation

**Date**: 2026-01-18

## Summary

Implemented a realistic Limelight simulation (`LimelightSim`) that simulates AprilTag detection based on robot position, camera FOV, and tag visibility. The simulation publishes to NetworkTables so the existing `LimelightEstimator` works unchanged.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/vision/LimelightSim.java` - Core simulation class
- `src/main/java/frc/robot/testbots/VisionSimTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `LimelightSim` config interface
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Integrated LimelightSim

## Implementation Details

### LimelightSim Features

1. **Visibility Calculation**:
   - Checks each AprilTag for distance (min/max bounds)
   - Verifies tag is within horizontal and vertical FOV
   - Checks tag orientation (dot product between camera forward and tag normal)

2. **Realistic Metrics**:
   - Calculates tx/ty angles in degrees
   - Computes tag area as percentage of image
   - Estimates ambiguity based on distance and viewing angle

3. **Noise Simulation**:
   - Gaussian noise on position (configurable std dev)
   - Gaussian noise on rotation (configurable std dev)
   - Random frame drops (configurable probability)

4. **NetworkTables Publishing**:
   - Publishes `botpose_wpiblue` (MegaTag1 format)
   - Publishes `botpose_orb_wpiblue` (MegaTag2 format)
   - Publishes `tv`, `tx`, `ty`, `ta`, `tid` for primary target
   - Array format matches LimelightHelpers expectations exactly

### Config Parameters Added

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LimelightSim/ForwardOffset` | 0.3 | Camera forward offset (meters) |
| `LimelightSim/SideOffset` | 0.0 | Camera side offset (meters) |
| `LimelightSim/Height` | 0.5 | Camera height (meters) |
| `LimelightSim/PitchDeg` | 20.0 | Camera pitch angle (degrees) |
| `LimelightSim/HorizontalFOV` | 63.3 | Horizontal FOV (degrees) |
| `LimelightSim/VerticalFOV` | 49.7 | Vertical FOV (degrees) |
| `LimelightSim/MaxDistance` | 5.0 | Max detection distance (meters) |
| `LimelightSim/MinDistance` | 0.3 | Min detection distance (meters) |
| `LimelightSim/MaxTagAngle` | 60.0 | Max tag viewing angle (degrees) |
| `LimelightSim/PositionNoise` | 0.02 | Position noise std dev (meters) |
| `LimelightSim/RotationNoise` | 1.0 | Rotation noise std dev (degrees) |
| `LimelightSim/LatencyMs` | 25.0 | Pipeline latency (ms) |
| `LimelightSim/FrameDropProb` | 0.05 | Frame drop probability |
| `LimelightSim/Enabled?` | true | Enable/disable simulation |

### VisionSimTestbot Controls

| Button | Action |
|--------|--------|
| Left Stick | Translate (field-relative) |
| Right Stick X | Rotate |
| START | Zero gyro heading |
| BACK | Reset pose to origin |
| A | Teleport to (2, 2) |
| B | Teleport to (8, 4) - field center |
| Y | Teleport to (14, 6) facing left |
| X | Lock wheels (X pattern) |

## Testing Done

- [x] `./gradlew build` - passed

## How to Test

1. Run simulation: `./gradlew simulateJava -Probot=VisionSimTestbot`
2. Open AdvantageScope, connect to localhost
3. Drive robot around field using controller
4. Observe:
   - `LimelightSim/TagCount` changes based on position
   - `Vision/` telemetry shows accepted/rejected estimates
   - 3D field shows Vision pose updating
   - tx/ty values change as robot moves relative to tags

## Algorithm: Tag Visibility

For each AprilTag on the field:
1. Calculate distance from camera to tag
2. Check within min/max detection range
3. Transform tag position to camera coordinate frame
4. Check tag is in front of camera (positive X)
5. Calculate horizontal/vertical angles, check within FOV
6. Check tag normal faces camera (dot product check)
7. Calculate area, ambiguity, tx/ty for visible tags

## Known Issues / TODO

- [ ] May want to add occlusion simulation (tags blocked by other objects)
- [ ] Could add motion blur simulation at high speeds
- [ ] Consider adding per-tag noise variation based on distance

## Notes for Next Session

The simulation uses the fused pose (`latestFusedPose`) as the ground truth for generating vision data. This creates a feedback loop that may make the simulation appear more stable than reality. For more realistic testing, consider using odometry-only pose or adding additional drift.
