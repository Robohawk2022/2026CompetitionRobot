# Session: Add Simulation Input Control to MCP Server

**Date**: 2026-01-23

## Summary

Added 4 new tools to the `frc-networktables` MCP server to control robot simulation inputs via the HALSim WebSocket protocol.

## Changes Made

### Files Modified
- `build.gradle` - Added `wpi.sim.addWebsocketsServer().defaultEnabled = true` to enable HALSim WebSocket server
- `.mcp-server/requirements.txt` - Added `websockets>=12.0` dependency
- `.mcp-server/frc_networktables_server.py` - Added WebSocket client and 4 new simulation control tools

## New MCP Tools

| Tool | Description |
|------|-------------|
| `simulation_control_status` | Check if HALSim WebSocket control is available |
| `set_robot_mode` | Enable/disable robot and set mode (teleop/autonomous/test) |
| `set_joystick_input` | Set Xbox controller axes, buttons, and D-pad |
| `set_alliance` | Set alliance color (red/blue) and station (1-3) |

## Technical Details

### WebSocket Protocol
- Connects to `ws://localhost:3300/wpilibws` (HALSim WebSocket server)
- Uses JSON messages with types: `DriverStation`, `Joystick`
- Auto-reconnects on connection loss

### Xbox Controller Mapping
Axes:
- 0: left_x, 1: left_y, 2: left_trigger, 3: right_trigger, 4: right_x, 5: right_y

Buttons (1-indexed):
- 1: A, 2: B, 3: X, 4: Y, 5: left_bumper, 6: right_bumper, 7: back, 8: start, 9: left_stick, 10: right_stick

### Example Usage

```python
# Enable teleop mode
mcp set_robot_mode --enabled true --mode teleop

# Drive forward with left stick
mcp set_joystick_input --joystick 0 --axes '{"left_y": -0.5}'

# Press A button
mcp set_joystick_input --joystick 1 --buttons '{"a": true}'

# Set alliance
mcp set_alliance --alliance red --station 1
```

## Testing Done

- [x] `./gradlew build` - passed
- [x] Python syntax check - passed
- [ ] Full simulation test - requires running simulation

## Verification Steps

To verify the implementation:
1. Start simulation: `./gradlew simulateJava`
2. Use MCP tool `simulation_control_status` to verify WebSocket connection
3. Use `set_robot_mode` with `enabled: true` and `mode: teleop`
4. Use `set_joystick_input` to drive the robot
5. Verify robot moves in simulation GUI

## Known Limitations

- **Simulation only** - Tools refuse to operate on real robot (WebSocket not available)
- **Requires build.gradle change** - WebSocket server must be enabled in build
- **Connection required** - WebSocket must connect before sending commands

## Notes for Next Session

- The WebSocket server port (3300) is the WPILib default and should not need changing
- If joystick inputs don't work, ensure the simulation has focus and keyboard control is disabled
- The alliance setting takes effect immediately in simulation
