package frc.robot.subsystems.vision;

/**
 * Points of Interest for AprilTag filtering.
 * <p>
 * Allows filtering vision estimates to only use tags at specific field locations.
 * This can improve accuracy when you know which part of the field you're near.
 */
public enum TagPOI {
    // Game element based
    TOWER,          // Central scoring/climbing structures (tags 1-12, 17-28)
    OUTPOST,        // Outpost scoring stations (tags 13-14, 29-30)
    HUB,            // Alliance hub/wall area (tags 15-16, 31-32)

    // Position based
    LEFT_SIDE,      // Left half of field (tags 17-32)
    RIGHT_SIDE,     // Right half of field (tags 1-16)
    CENTER,         // Center field elements (tags 1-12, 17-28)
    ALLIANCE_WALL,  // Field edge tags (tags 13-16, 29-32)

    // No filtering
    ALL             // Use all tags
}
