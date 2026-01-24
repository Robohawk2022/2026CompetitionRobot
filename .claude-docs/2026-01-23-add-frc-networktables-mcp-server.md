# Session: Add FRC NetworkTables MCP Server

**Date**: 2026-01-23

## Summary

Created an MCP (Model Context Protocol) server that connects to the robot's NetworkTables, allowing Claude to query live robot state during simulation. Also documented MCP servers and simulation detection in CLAUDE.md.

## Changes Made

### Files Created
- `mcp-server/frc_networktables_server.py` - Python MCP server that connects to NetworkTables
- `mcp-server/requirements.txt` - Python dependencies (pyntcore, mcp)
- `mcp-server/setup.sh` - Setup script to create venv and install dependencies
- `mcp-server/README.md` - Documentation for using the MCP server
- `.mcp.json` - Claude Code MCP server configuration

### Files Modified
- `CLAUDE.md` - Added two new documentation sections:
  1. **MCP Servers (AI Tools)** section (after "Workflow for AI Agents") - Documents frc-networktables and frc-docs servers with tool tables and setup instructions
  2. **Simulation Detection** section (in "Debugging Tips") - Documents `Robot.isSimulation()`, alliance detection in sim, and MCP connection verification

## MCP Server Tools

The server provides these tools:

| Tool | Description |
|------|-------------|
| `get_robot_state` | Enabled/disabled, mode, alliance |
| `get_robot_pose` | Position (x, y in feet) and heading (degrees) |
| `get_swerve_status` | Velocities, mode, turbo/sniper, module states |
| `get_vision_status` | Tag detection, pose estimate, quality metrics |
| `get_active_commands` | Currently running commands from CommandLogger |
| `get_odometry_diagnostics` | Drift, vision accept rate, rejection breakdown |
| `get_subsystem_status` | Query any subsystem by name |
| `get_preference` | Get a config value |
| `list_preferences` | List all preferences (optionally filtered by category) |
| `get_networktables_value` | Get any NT value by full path |
| `list_networktables` | List entries in a table |
| `connection_status` | Check NT connection status |

## How to Use

1. Install dependencies:
   ```bash
   cd mcp-server
   ./setup.sh
   ```

2. Start robot simulation:
   ```bash
   ./gradlew simulateJava
   ```

3. Start Claude Code - it will prompt to enable the `frc-networktables` MCP server

4. Ask Claude questions like:
   - "What is the robot's current pose?"
   - "Show me the swerve drive status"
   - "What vision tags are detected?"
   - "Get the odometry diagnostics"

## Testing Done

- [x] MCP server files created
- [x] Configuration added to .mcp.json
- [ ] Not yet tested with running simulation (requires `pyntcore` installation)

## Known Issues / TODO

- [ ] The MCP server requires Python 3.10+ and `pyntcore` from RobotPy
- [ ] May need to adjust connection timeout or retry logic
- [ ] Could add `set_preference` tool to modify config values live

## Notes for Next Session

The MCP server uses the `pyntcore` library from RobotPy to connect to NetworkTables. It connects to `localhost:5810` by default (WPILib simulation NetworkTables port).

If the user has issues with pyntcore installation, they may need to:
```bash
pip install robotpy
# or specifically
pip install pyntcore
```

The server reads from the same NetworkTables paths that AdvantageScope uses, so data visibility should be identical.
