# FRC NetworkTables MCP Server

An MCP (Model Context Protocol) server that connects to an FRC robot's NetworkTables, allowing Claude to query live robot state during simulation or competition.

## Features

- **Live Robot State**: Query pose, velocity, heading, enabled state
- **Swerve Drive Status**: Module angles, velocities, drive modes
- **Vision Diagnostics**: Tag detection, pose estimates, quality metrics
- **Odometry Health**: Drift tracking, vision acceptance rates
- **Command Logger**: Active commands, recent activity
- **Preferences**: Read robot configuration values

## Prerequisites

- Python 3.10+
- WPILib robot code running in simulation (or real robot)

## Installation

1. Run the setup script:
   ```bash
   cd mcp-server
   chmod +x setup.sh
   ./setup.sh
   ```

2. The MCP server is already configured in `.mcp.json` at the project root. When you start Claude Code, it will prompt you to enable it.

   If you need to configure manually, create `.mcp.json` in your project root:
   ```json
   {
     "mcpServers": {
       "frc-networktables": {
         "command": "python3",
         "args": ["mcp-server/frc_networktables_server.py"]
       }
     }
   }
   ```

## Usage

1. Start the robot simulation:
   ```bash
   ./gradlew simulateJava
   ```

2. Start Claude Code - the MCP server will connect automatically

3. Ask Claude about the robot:
   - "What is the robot's current pose?"
   - "Show me the swerve drive status"
   - "What vision tags are detected?"
   - "Get the odometry diagnostics"

## Available Tools

| Tool | Description |
|------|-------------|
| `get_robot_state` | Enabled/disabled, mode, alliance |
| `get_robot_pose` | Position (x, y in feet) and heading (degrees) |
| `get_swerve_status` | Velocities, mode, turbo/sniper, module states |
| `get_vision_status` | Tag detection, pose estimate, quality |
| `get_active_commands` | Currently running commands |
| `get_odometry_diagnostics` | Drift, vision accept rate, rejections |
| `get_subsystem_status` | Query any subsystem by name |
| `get_preference` | Get a config value |
| `list_preferences` | List all preferences |
| `get_networktables_value` | Get any NT value by path |
| `list_networktables` | List entries in a table |
| `connection_status` | Check NT connection |

## Manual Testing

Run the server directly:
```bash
source venv/bin/activate
python frc_networktables_server.py --host localhost --port 5810
```

## Troubleshooting

**"Not connected to robot"**
- Make sure the simulation is running (`./gradlew simulateJava`)
- Check that NetworkTables is accessible on port 5810

**"Subsystem not found"**
- Verify the subsystem name matches what's published to SmartDashboard
- Check `/SmartDashboard/` in AdvantageScope to see available tables
