# Session: Fix HALSim WebSocket Simulation Control

**Date**: 2026-01-23

## Summary

Fixed the MCP server's HALSim WebSocket control by adding the critical `">new_data": true` flag to DriverStation messages. This flag signals to the robot code that fresh joystick data is available and should be read.

## Problem

The MCP server's simulation control tools (`set_robot_mode`, `set_joystick_input`, `drive_robot`) were sending WebSocket messages correctly, but the robot code was not responding to joystick inputs. The robot always showed `LeftY=0.00` regardless of the values being sent.

## Root Cause

According to the [WPILib HALSim WebSocket Protocol](https://github.com/wpilibsuite/allwpilib/blob/main/simulation/halsim_ws_core/doc/hardware_ws_api.md):

> "Joystick updates remain invisible to robot code until the DriverStation sends a message with `">new_data"` set to true, enabling synchronous state updates."

The MCP server was sending joystick data but was NOT sending the `">new_data"` flag, so the robot code never saw the updated values.

## Solution

Modified `.mcp-server/frc_networktables_server.py`:

1. Added `">new_data": True` to `set_driver_station_mode()`:
```python
async def set_driver_station_mode(enabled: bool, autonomous: bool = False, test: bool = False) -> bool:
    message = {
        "type": "DriverStation",
        "device": "",
        "data": {
            ">enabled": enabled,
            ">autonomous": autonomous,
            ">test": test,
            ">new_data": True  # Critical: tells robot code to read fresh joystick data
        }
    }
    return await send_ws_message(message)
```

2. Added `signal_new_data()` helper function:
```python
async def signal_new_data() -> bool:
    """Signal to the robot code that new joystick data is available."""
    message = {
        "type": "DriverStation",
        "device": "",
        "data": {
            ">new_data": True
        }
    }
    return await send_ws_message(message)
```

3. Modified `set_joystick_state()` to call `signal_new_data()` after sending joystick data

4. Updated `drive_robot()` to:
   - Send updates at 50Hz (matching robot loop rate) instead of 20Hz
   - Re-send enabled state with each joystick update

## Changes Made

### Files Modified
- `.mcp-server/frc_networktables_server.py` - Added new_data signaling to WebSocket messages

## Testing Done

- [x] `./gradlew simulateJava -Probot=SwerveTestbot` - Simulation starts correctly
- [x] WebSocket control test - Robot enters teleop mode
- [x] Joystick input test - Robot sees correct controller values (`LeftY=-0.50`)
- [x] Movement test - Robot moves ~225 feet in 3 seconds when driving forward

## Key Protocol Details

The HALSim WebSocket protocol uses these prefixes:
- `">"` - Values written TO the robot (inputs)
- `"<"` - Values read FROM the robot (outputs)

Important DriverStation fields:
- `">enabled"` - Enables/disables robot
- `">autonomous"` / `">test"` - Mode selection
- `">new_data"` - **One-shot signal** indicating fresh joystick/DS data is available

The correct sequence for controlling the robot:
1. Send Joystick message with `">axes"`, `">buttons"`, `">povs"`
2. Send DriverStation message with `">new_data": true`
3. Repeat at ~50Hz for continuous control

## Notes for Next Session

- The MCP `drive_robot` tool now works correctly for simulation control
- Consider adding a `rotation` parameter to `drive_robot` for testing rotation
- The robot's default controller mapping is 8BitDo mode (not Xbox) - this may affect axis interpretation
