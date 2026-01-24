#!/usr/bin/env python3
"""
FRC NetworkTables MCP Server

Connects to a robot's NetworkTables server (simulation or real) and provides
tools for querying robot state, pose, subsystem status, and more.

Usage:
    python frc_networktables_server.py [--host HOST] [--port PORT]

Default connects to localhost:5810 (WPILib simulation default)
"""

import argparse
import asyncio
import json
import socket
import sys
import time
from typing import Any

import ntcore
import websockets
from websockets.exceptions import ConnectionClosed, WebSocketException
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool, TextContent

# Global NetworkTables instance
_nt_inst: ntcore.NetworkTableInstance | None = None
_connected = False
_current_host = "localhost"
_current_port = 5810
_multi_subscriber = None  # Subscribe to all topics

# HALSim WebSocket state (for simulation control)
_ws_connection = None
_ws_host = "localhost"
_ws_port = 3300
_ws_connected = False
_joysticks_initialized = set()  # Track which joysticks have been initialized

# Xbox controller axis indices
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 3
AXIS_RIGHT_X = 4
AXIS_RIGHT_Y = 5

# Xbox controller button indices (1-indexed in WPILib)
BUTTON_A = 1
BUTTON_B = 2
BUTTON_X = 3
BUTTON_Y = 4
BUTTON_LEFT_BUMPER = 5
BUTTON_RIGHT_BUMPER = 6
BUTTON_BACK = 7
BUTTON_START = 8
BUTTON_LEFT_STICK = 9
BUTTON_RIGHT_STICK = 10


def get_nt() -> ntcore.NetworkTableInstance:
    """Get the NetworkTables instance."""
    global _nt_inst
    if _nt_inst is None:
        _nt_inst = ntcore.NetworkTableInstance.getDefault()
    return _nt_inst


def connect_to_robot(host: str = "localhost", port: int = 5810) -> bool:
    """Connect to robot NetworkTables server."""
    global _connected, _current_host, _current_port, _multi_subscriber
    nt = get_nt()

    _current_host = host
    _current_port = port

    # Start client if not already connected
    if not _connected:
        nt.startClient4("MCP-FRC-Client")
        nt.setServer(host, port)
        _connected = True

        # Subscribe to ALL topics using MultiSubscriber (like Shuffleboard does)
        # This ensures we receive all published values
        # Subscribe to everything with "" prefix (all topics)
        _multi_subscriber = ntcore.MultiSubscriber(nt, [""])

        # Wait briefly for connection and initial data
        time.sleep(1.5)
    else:
        # Already started, just update the server
        nt.setServer(host, port)
        time.sleep(0.5)

    return nt.isConnected()


def scan_port(host: str, port: int, timeout: float = 0.3) -> bool:
    """Check if a port is open on a host."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except (socket.error, socket.timeout):
        return False


def discover_networktables_servers() -> list[dict[str, Any]]:
    """
    Scan for NetworkTables servers on common hosts/ports.
    Returns a list of found servers with connection info.
    """
    found_servers = []

    # Common NetworkTables ports
    # 5810 = NT4 default (simulation and robot)
    # 1735 = NT3 default (older)
    ports = [5810, 1735]

    # Common hosts to check
    hosts_to_check = [
        ("localhost", "Local simulation"),
        ("127.0.0.1", "Local simulation (IP)"),
        ("10.33.73.2", "roboRIO (Team 3373)"),
        ("172.22.11.2", "roboRIO via USB"),
        ("roborio-3373-frc.local", "roboRIO (mDNS)"),
    ]

    for host, description in hosts_to_check:
        for port in ports:
            if scan_port(host, port):
                found_servers.append({
                    "host": host,
                    "port": port,
                    "description": description,
                    "protocol": "NT4" if port == 5810 else "NT3"
                })

    return found_servers


# =============================================================================
# HALSim WebSocket Client (for simulation input control)
# =============================================================================

async def connect_websocket(host: str = "localhost", port: int = 3300) -> bool:
    """Connect to the HALSim WebSocket server."""
    global _ws_connection, _ws_host, _ws_port, _ws_connected, _joysticks_initialized

    _ws_host = host
    _ws_port = port

    try:
        uri = f"ws://{host}:{port}/wpilibws"
        _ws_connection = await asyncio.wait_for(
            websockets.connect(uri),
            timeout=2.0
        )
        _ws_connected = True
        _joysticks_initialized = set()  # Reset on new connection
        return True
    except (WebSocketException, OSError, asyncio.TimeoutError) as e:
        _ws_connected = False
        _ws_connection = None
        return False


async def disconnect_websocket():
    """Disconnect from the HALSim WebSocket server."""
    global _ws_connection, _ws_connected

    if _ws_connection:
        try:
            await _ws_connection.close()
        except Exception:
            pass
    _ws_connection = None
    _ws_connected = False


async def send_ws_message(message: dict) -> bool:
    """Send a message to the HALSim WebSocket server."""
    global _ws_connection, _ws_connected

    if not _ws_connection or not _ws_connected:
        # Try to reconnect
        if not await connect_websocket(_ws_host, _ws_port):
            return False

    try:
        await _ws_connection.send(json.dumps(message))
        return True
    except (ConnectionClosed, WebSocketException) as e:
        _ws_connected = False
        return False


def is_simulation() -> bool:
    """Check if we're connected to a simulation (not a real robot)."""
    # WebSocket control is only available in simulation
    # We can check this by seeing if port 3300 is open on localhost
    return scan_port(_ws_host, _ws_port, timeout=0.1)


async def set_driver_station_mode(enabled: bool, autonomous: bool = False, test: bool = False) -> bool:
    """Set the driver station mode."""
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


async def signal_new_data() -> bool:
    """
    Signal to the robot code that new joystick data is available.
    Must be called after set_joystick_state() for the robot to see the values.
    """
    message = {
        "type": "DriverStation",
        "device": "",
        "data": {
            ">new_data": True
        }
    }
    return await send_ws_message(message)


async def set_alliance_station(alliance: str, station: int = 1) -> bool:
    """Set the alliance color and station."""
    # Station format: "red1", "red2", "red3", "blue1", "blue2", "blue3"
    station_str = f"{alliance.lower()}{station}"
    message = {
        "type": "DriverStation",
        "device": "",
        "data": {
            ">station": station_str
        }
    }
    return await send_ws_message(message)


async def init_xbox_controller(port: int = 0, name: str = "Claude's Controller") -> bool:
    """
    Initialize a virtual Xbox controller on the specified port.
    This must be called before the joystick values will be accepted.
    """
    global _joysticks_initialized

    if port in _joysticks_initialized:
        return True  # Already initialized

    # Send joystick descriptor - this tells HALSim about the controller
    descriptor_msg = {
        "type": "Joystick",
        "device": str(port),
        "data": {
            ">name": name,
            ">type": 1,  # 1 = Xbox controller
            ">isXbox": True,
            ">axisCount": 6,
            ">buttonCount": 12,
            ">povCount": 1,
            # Initialize with default values
            ">axes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ">buttons": [False] * 12,
            ">povs": [-1]
        }
    }

    success = await send_ws_message(descriptor_msg)
    if success:
        _joysticks_initialized.add(port)
    return success


async def set_joystick_state(
    port: int,
    axes: list[float] | None = None,
    buttons: list[bool] | None = None,
    povs: list[int] | None = None
) -> bool:
    """
    Set joystick state.

    Args:
        port: Joystick port (0-5)
        axes: List of 6 axis values (-1.0 to 1.0 for sticks, 0.0 to 1.0 for triggers)
        buttons: List of 12 button states (index 0 is button 1)
        povs: List of POV values (-1 = unpressed, 0-359 for direction)
    """
    # Auto-initialize the controller if not done yet
    if port not in _joysticks_initialized:
        await init_xbox_controller(port)

    data = {}

    if axes is not None:
        # Ensure we have exactly 6 axes
        while len(axes) < 6:
            axes.append(0.0)
        data[">axes"] = axes[:6]

    if buttons is not None:
        # Ensure we have enough buttons (12 for Xbox)
        while len(buttons) < 12:
            buttons.append(False)
        data[">buttons"] = buttons[:12]

    if povs is not None:
        data[">povs"] = povs

    if not data:
        return False

    message = {
        "type": "Joystick",
        "device": str(port),
        "data": data
    }
    success = await send_ws_message(message)
    if success:
        # Signal new data available - this is CRITICAL for robot code to see the values
        await signal_new_data()
    return success


def get_table_values(table_path: str) -> dict[str, Any]:
    """Get all values from a NetworkTables subtable."""
    nt = get_nt()
    table = nt.getTable(table_path)
    result = {}

    for key in table.getKeys():
        topic = table.getTopic(key)
        entry = topic.genericSubscribe()
        value = entry.get()
        if value is not None:
            result[key] = value.value()

    # Also get subtables recursively (one level)
    for subtable in table.getSubTables():
        subtable_path = f"{table_path}/{subtable}"
        sub_values = {}
        st = nt.getTable(subtable_path)
        for key in st.getKeys():
            topic = st.getTopic(key)
            entry = topic.genericSubscribe()
            value = entry.get()
            if value is not None:
                sub_values[key] = value.value()
        if sub_values:
            result[subtable] = sub_values

    return result


def get_all_topics() -> list[str]:
    """Get all published topic names in NetworkTables."""
    nt = get_nt()
    topics = nt.getTopics()
    return [t.getName() for t in topics]


def get_topics_under_path(path: str) -> dict[str, Any]:
    """Get all topics and their values under a given path prefix."""
    nt = get_nt()
    result = {}
    path_prefix = path if path.startswith("/") else f"/{path}"
    if not path_prefix.endswith("/"):
        path_prefix += "/"

    for topic in nt.getTopics():
        name = topic.getName()
        if name.startswith(path_prefix):
            # Get the key relative to the path
            relative_key = name[len(path_prefix):]
            entry = topic.genericSubscribe()
            value = entry.get()
            if value is not None:
                result[relative_key] = value.value()

    return result


def format_pose(x: float, y: float, rotation: float) -> str:
    """Format pose for display."""
    return f"Position: ({x:.2f}, {y:.2f}) ft, Heading: {rotation:.1f} deg"


# Create MCP server
server = Server("frc-networktables")


@server.list_tools()
async def list_tools() -> list[Tool]:
    """List available tools."""
    return [
        Tool(
            name="get_robot_state",
            description="Get the robot's current state including enabled/disabled, mode (auto/teleop), and alliance",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_robot_pose",
            description="Get the robot's current position and heading on the field in feet and degrees",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_swerve_status",
            description="Get swerve drive status including velocity, heading, and module states",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_vision_status",
            description="Get vision/Limelight status including tag detection, pose estimates, and quality metrics",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_active_commands",
            description="Get currently running commands and recent command activity",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_odometry_diagnostics",
            description="Get odometry health metrics including drift, vision acceptance rate, and rejection reasons",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="get_subsystem_status",
            description="Get status of a specific subsystem by name",
            inputSchema={
                "type": "object",
                "properties": {
                    "subsystem": {
                        "type": "string",
                        "description": "Subsystem name (e.g., 'Swerve', 'Shooter', 'Vision')"
                    }
                },
                "required": ["subsystem"]
            }
        ),
        Tool(
            name="get_preference",
            description="Get a preference/config value from the robot",
            inputSchema={
                "type": "object",
                "properties": {
                    "key": {
                        "type": "string",
                        "description": "Preference key (e.g., 'Swerve/kP', 'Vision/MaxDistance')"
                    }
                },
                "required": ["key"]
            }
        ),
        Tool(
            name="list_preferences",
            description="List all available preferences/config values",
            inputSchema={
                "type": "object",
                "properties": {
                    "category": {
                        "type": "string",
                        "description": "Optional category to filter (e.g., 'Swerve', 'Vision')"
                    }
                },
                "required": []
            }
        ),
        Tool(
            name="get_networktables_value",
            description="Get a raw NetworkTables value by path",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {
                        "type": "string",
                        "description": "Full NetworkTables path (e.g., '/SmartDashboard/Swerve/Pose X')"
                    }
                },
                "required": ["path"]
            }
        ),
        Tool(
            name="list_networktables",
            description="List all NetworkTables entries under a path",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {
                        "type": "string",
                        "description": "NetworkTables path to list (default: SmartDashboard)"
                    }
                },
                "required": []
            }
        ),
        Tool(
            name="list_all_topics",
            description="List ALL topics currently published to NetworkTables (like Shuffleboard sees)",
            inputSchema={
                "type": "object",
                "properties": {
                    "filter": {
                        "type": "string",
                        "description": "Optional filter string to match topic names (e.g., 'Swerve', 'Vision')"
                    }
                },
                "required": []
            }
        ),
        Tool(
            name="connection_status",
            description="Check if connected to the robot's NetworkTables server",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="scan_for_servers",
            description="Scan for NetworkTables servers on the network (localhost, roboRIO, USB). Use this to find where the simulation or robot is running.",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="reconnect",
            description="Reconnect to a different NetworkTables server. Use after scan_for_servers to connect to a discovered server.",
            inputSchema={
                "type": "object",
                "properties": {
                    "host": {
                        "type": "string",
                        "description": "Host to connect to (e.g., 'localhost', '10.33.73.2', 'roborio-3373-frc.local')"
                    },
                    "port": {
                        "type": "integer",
                        "description": "Port number (default: 5810 for NT4)"
                    }
                },
                "required": ["host"]
            }
        ),
        # Simulation control tools (HALSim WebSocket)
        Tool(
            name="simulation_control_status",
            description="Check if simulation control is available (HALSim WebSocket). This is only available when running in simulation, not on a real robot.",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        Tool(
            name="set_robot_mode",
            description="Enable/disable the robot and set its mode. SIMULATION ONLY - will not work on a real robot.",
            inputSchema={
                "type": "object",
                "properties": {
                    "enabled": {
                        "type": "boolean",
                        "description": "Whether the robot should be enabled"
                    },
                    "mode": {
                        "type": "string",
                        "enum": ["teleop", "autonomous", "test"],
                        "description": "Robot mode (default: teleop)"
                    }
                },
                "required": ["enabled"]
            }
        ),
        Tool(
            name="set_joystick_input",
            description="Set Xbox controller inputs for simulation. SIMULATION ONLY. Axes range from -1.0 to 1.0 (sticks) or 0.0 to 1.0 (triggers). POV is -1 (unpressed) or 0-359 degrees.",
            inputSchema={
                "type": "object",
                "properties": {
                    "joystick": {
                        "type": "integer",
                        "description": "Joystick port (0=driver, 1=operator)"
                    },
                    "axes": {
                        "type": "object",
                        "description": "Axis values: left_x, left_y, right_x, right_y, left_trigger, right_trigger",
                        "properties": {
                            "left_x": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                            "left_y": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                            "right_x": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                            "right_y": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                            "left_trigger": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                            "right_trigger": {"type": "number", "minimum": 0.0, "maximum": 1.0}
                        }
                    },
                    "buttons": {
                        "type": "object",
                        "description": "Button states: a, b, x, y, left_bumper, right_bumper, back, start, left_stick, right_stick",
                        "properties": {
                            "a": {"type": "boolean"},
                            "b": {"type": "boolean"},
                            "x": {"type": "boolean"},
                            "y": {"type": "boolean"},
                            "left_bumper": {"type": "boolean"},
                            "right_bumper": {"type": "boolean"},
                            "back": {"type": "boolean"},
                            "start": {"type": "boolean"},
                            "left_stick": {"type": "boolean"},
                            "right_stick": {"type": "boolean"}
                        }
                    },
                    "pov": {
                        "type": "integer",
                        "description": "D-pad direction: -1 (unpressed), 0 (up), 45, 90 (right), 135, 180 (down), 225, 270 (left), 315"
                    }
                },
                "required": ["joystick"]
            }
        ),
        Tool(
            name="set_alliance",
            description="Set the alliance color and station. SIMULATION ONLY.",
            inputSchema={
                "type": "object",
                "properties": {
                    "alliance": {
                        "type": "string",
                        "enum": ["red", "blue"],
                        "description": "Alliance color"
                    },
                    "station": {
                        "type": "integer",
                        "description": "Driver station number (1, 2, or 3)"
                    }
                },
                "required": ["alliance"]
            }
        ),
        Tool(
            name="drive_robot",
            description="Drive the robot in simulation by continuously sending joystick inputs for a specified duration. "
                        "Automatically enables teleop mode. SIMULATION ONLY. "
                        "Use left_y=-0.5 to drive forward, right_x=0.5 to rotate right. "
                        "Can also hold buttons like left_bumper for special modes (e.g., orbit).",
            inputSchema={
                "type": "object",
                "properties": {
                    "left_x": {
                        "type": "number",
                        "minimum": -1.0,
                        "maximum": 1.0,
                        "description": "Left stick X axis (strafe left/right)"
                    },
                    "left_y": {
                        "type": "number",
                        "minimum": -1.0,
                        "maximum": 1.0,
                        "description": "Left stick Y axis (forward/backward, negative=forward)"
                    },
                    "right_x": {
                        "type": "number",
                        "minimum": -1.0,
                        "maximum": 1.0,
                        "description": "Right stick X axis (rotation)"
                    },
                    "duration": {
                        "type": "number",
                        "minimum": 0.1,
                        "maximum": 10.0,
                        "description": "Duration to drive in seconds (default: 2.0, max: 10.0)"
                    },
                    "buttons": {
                        "type": "object",
                        "description": "Buttons to hold: a, b, x, y, left_bumper, right_bumper, back, start",
                        "properties": {
                            "a": {"type": "boolean"},
                            "b": {"type": "boolean"},
                            "x": {"type": "boolean"},
                            "y": {"type": "boolean"},
                            "left_bumper": {"type": "boolean"},
                            "right_bumper": {"type": "boolean"},
                            "back": {"type": "boolean"},
                            "start": {"type": "boolean"}
                        }
                    }
                },
                "required": []
            }
        )
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict[str, Any]) -> list[TextContent]:
    """Handle tool calls."""
    nt = get_nt()

    # Check connection (allow some tools to work without connection)
    # Simulation control tools also work without NT connection (they use WebSocket)
    connection_optional_tools = [
        "connection_status", "scan_for_servers", "reconnect",
        "simulation_control_status", "set_robot_mode", "set_joystick_input", "set_alliance",
        "drive_robot"
    ]
    if not nt.isConnected() and name not in connection_optional_tools:
        return [TextContent(
            type="text",
            text=f"Not connected to robot at {_current_host}:{_current_port}.\n"
                 f"Use 'scan_for_servers' to find available servers, or 'reconnect' to try a different host.\n"
                 f"Make sure the simulation is running (./gradlew simulateJava) and try again."
        )]

    try:
        if name == "connection_status":
            connected = nt.isConnected()
            connections = nt.getConnections()
            conn_info = [f"  - {c.remote_id} ({c.remote_ip})" for c in connections]

            status_lines = [
                f"Target: {_current_host}:{_current_port}",
                f"Connected: {connected}",
            ]

            if conn_info:
                status_lines.append("Active connections:")
                status_lines.extend(conn_info)
            else:
                status_lines.append("No active connections")

            # Add helpful hints if not connected
            if not connected:
                status_lines.append("")
                status_lines.append("Tip: Use 'scan_for_servers' to find available NetworkTables servers")
                status_lines.append("     Use 'reconnect' to connect to a different host/port")

            return [TextContent(type="text", text="\n".join(status_lines))]

        elif name == "scan_for_servers":
            servers = discover_networktables_servers()
            if servers:
                result = {
                    "found": len(servers),
                    "servers": servers,
                    "current_target": f"{_current_host}:{_current_port}",
                    "hint": "Use 'reconnect' tool with host and port to connect to a server"
                }
            else:
                result = {
                    "found": 0,
                    "servers": [],
                    "current_target": f"{_current_host}:{_current_port}",
                    "hint": "No servers found. Make sure the simulation is running (./gradlew simulateJava) or the robot is on."
                }
            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "reconnect":
            host = arguments.get("host", "localhost")
            port = arguments.get("port", 5810)

            # Attempt to reconnect
            success = connect_to_robot(host, port)

            result = {
                "target": f"{host}:{port}",
                "connected": success,
                "message": f"Successfully connected to {host}:{port}" if success else f"Failed to connect to {host}:{port}. Server may not be running."
            }
            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_robot_state":
            fms = get_table_values("FMSInfo")
            result = {
                "connected": nt.isConnected(),
                "fms_attached": fms.get("FMSControlData", 0) > 0,
                "alliance": "Red" if fms.get("IsRedAlliance", False) else "Blue",
                "match_number": fms.get("MatchNumber", 0),
                "event_name": fms.get("EventName", ""),
            }

            # Try to get robot state from different sources
            ds_table = get_table_values("DriverStation")
            if ds_table:
                result["enabled"] = ds_table.get("Enabled", False)
                result["autonomous"] = ds_table.get("Autonomous", False)
                result["teleop"] = ds_table.get("Teleop", False)

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_robot_pose":
            # Try multiple possible paths for swerve data
            swerve = get_table_values("SmartDashboard/SwerveSubsystem")
            if not swerve:
                swerve = get_table_values("SmartDashboard/Swerve")
            result = {}

            if swerve:
                # Handle both naming conventions (PoseXFeet vs "Pose X")
                x = swerve.get("PoseXFeet", swerve.get("Pose X", 0))
                y = swerve.get("PoseYFeet", swerve.get("Pose Y", 0))
                heading = swerve.get("HeadingDeg", swerve.get("Heading", 0))
                result = {
                    "x_feet": round(x, 2),
                    "y_feet": round(y, 2),
                    "heading_degrees": round(heading, 1),
                    "formatted": format_pose(x, y, heading)
                }
            else:
                result = {"error": "Swerve data not found. Looking in SmartDashboard/SwerveSubsystem and SmartDashboard/Swerve"}

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_swerve_status":
            # Try multiple possible paths for swerve data
            swerve = get_table_values("SmartDashboard/SwerveSubsystem")
            if not swerve:
                swerve = get_table_values("SmartDashboard/Swerve")
            if not swerve:
                return [TextContent(type="text", text="Swerve subsystem not found. Looking in SmartDashboard/SwerveSubsystem and SmartDashboard/Swerve")]

            # Handle both naming conventions
            result = {
                "mode": swerve.get("Mode", "unknown"),
                "enabled": swerve.get("Enabled?", False),
                "turbo_active": swerve.get("TurboActive?", swerve.get("Turbo?", False)),
                "sniper_active": swerve.get("SniperActive?", swerve.get("Sniper?", False)),
                "pose": {
                    "x_feet": round(swerve.get("PoseXFeet", swerve.get("Pose X", 0)), 2),
                    "y_feet": round(swerve.get("PoseYFeet", swerve.get("Pose Y", 0)), 2),
                    "heading_deg": round(swerve.get("HeadingDeg", swerve.get("Heading", 0)), 1)
                },
                "velocity": {
                    "vx_fps": round(swerve.get("Speedx", swerve.get("vx", 0)), 2),
                    "vy_fps": round(swerve.get("Speedy", swerve.get("vy", 0)), 2),
                    "omega_dps": round(swerve.get("SpeedO", swerve.get("omega", 0)), 1)
                },
                "reset_count": swerve.get("ResetCount", 0)
            }

            # Add module info if available (check Swerve/Module-XX path)
            modules = get_table_values("SmartDashboard/Swerve")
            for module in ["Module-FL", "Module-FR", "Module-BL", "Module-BR"]:
                if modules and module in modules:
                    result[module] = modules[module]

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_vision_status":
            # Vision publishes to SmartDashboard with "Vision/" prefix keys
            # Try both as a subtable and as prefixed keys in SmartDashboard
            vision = get_table_values("SmartDashboard/Vision")

            # Also check for keys directly in SmartDashboard with Vision/ prefix
            sd = get_table_values("SmartDashboard")
            vision_from_sd = {}
            if sd:
                for key, value in sd.items():
                    if key.startswith("Vision/"):
                        vision_from_sd[key.replace("Vision/", "")] = value

            # Merge both sources, preferring the subtable
            if vision_from_sd and not vision:
                vision = vision_from_sd
            elif vision_from_sd:
                vision.update(vision_from_sd)

            if not vision:
                return [TextContent(type="text", text="Vision data not found. Looking in SmartDashboard/Vision")]

            result = {
                "valid": vision.get("Valid", False),
                "status": vision.get("Status", "unknown"),
                "active_poi": vision.get("ActivePOI", "none"),
                "tag_count": vision.get("TagCount", 0),
                "avg_tag_distance": round(vision.get("AvgTagDist", 0), 2),
                "std_dev_xy": round(vision.get("StdDevXY", 0), 3),
                "is_megatag2": vision.get("IsMegaTag2", False),
                "rejection_reason": vision.get("RejectionReason", "")
            }

            if vision.get("Valid", False):
                result["pose"] = {
                    "x_meters": round(vision.get("PoseX", vision.get("Pose X", 0)), 2),
                    "y_meters": round(vision.get("PoseY", vision.get("Pose Y", 0)), 2),
                    "rotation_deg": round(vision.get("PoseRot", vision.get("Rotation", 0)), 1)
                }

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_active_commands":
            cmd_logger = get_table_values("SmartDashboard/CommandLogger")
            result = cmd_logger if cmd_logger else {"status": "CommandLogger not found"}
            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_odometry_diagnostics":
            odom = get_table_values("SmartDashboard/Odom")
            if not odom:
                return [TextContent(type="text", text="Odometry diagnostics not found")]

            result = {
                "drift": {
                    "current_m": round(odom.get("DriftCurrent_m", 0), 3),
                    "max_m": round(odom.get("DriftMax_m", 0), 3),
                    "avg_m": round(odom.get("DriftAvg_m", 0), 3)
                },
                "vision": {
                    "accept_rate_pct": round(odom.get("VisionAcceptRate_%", 0), 1),
                    "latency_ms": round(odom.get("VisionLatency_ms", 0), 1),
                    "accepted": odom.get("VisionAccepted", 0),
                    "rejected": odom.get("VisionRejected", 0)
                }
            }

            # Add rejection breakdown if available
            reject_keys = [k for k in odom.keys() if k.startswith("Reject/")]
            if reject_keys:
                result["rejections"] = {k.replace("Reject/", ""): odom[k] for k in reject_keys}

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "get_subsystem_status":
            subsystem = arguments.get("subsystem", "")
            data = get_table_values(f"SmartDashboard/{subsystem}")
            if not data:
                # Try without SmartDashboard prefix
                data = get_table_values(subsystem)
            if not data:
                return [TextContent(type="text", text=f"Subsystem '{subsystem}' not found")]
            return [TextContent(type="text", text=json.dumps(data, indent=2))]

        elif name == "get_preference":
            key = arguments.get("key", "")
            prefs = nt.getTable("Preferences")
            topic = prefs.getTopic(key)
            entry = topic.genericSubscribe()
            value = entry.get()
            if value is not None:
                return [TextContent(type="text", text=f"{key} = {value.value()}")]
            return [TextContent(type="text", text=f"Preference '{key}' not found")]

        elif name == "list_preferences":
            category = arguments.get("category", "")
            prefs = get_table_values("Preferences")

            if category:
                # Filter to category
                filtered = {}
                for key, value in prefs.items():
                    if key.startswith(category) or key == category:
                        filtered[key] = value
                prefs = filtered

            return [TextContent(type="text", text=json.dumps(prefs, indent=2))]

        elif name == "get_networktables_value":
            path = arguments.get("path", "")
            # Parse the path
            parts = path.strip("/").split("/")
            if len(parts) < 2:
                return [TextContent(type="text", text=f"Invalid path: {path}")]

            table_path = "/".join(parts[:-1])
            key = parts[-1]

            table = nt.getTable(table_path)
            topic = table.getTopic(key)
            entry = topic.genericSubscribe()
            value = entry.get()

            if value is not None:
                return [TextContent(type="text", text=f"{path} = {value.value()}")]
            return [TextContent(type="text", text=f"Value not found at '{path}'")]

        elif name == "list_networktables":
            path = arguments.get("path", "SmartDashboard")

            # Get all topics and find ones under this path
            all_topics = get_all_topics()
            path_prefix = f"/{path}" if not path.startswith("/") else path

            # Filter topics under this path
            matching_topics = [t for t in all_topics if t.startswith(path_prefix)]

            # Also try the table API
            table = nt.getTable(path)
            table_keys = list(table.getKeys())
            subtables = list(table.getSubTables())

            # Get values using the topics approach
            values = get_topics_under_path(path)

            result = {
                "path": path,
                "topics_found": len(matching_topics),
                "sample_topics": matching_topics[:20] if matching_topics else [],
                "table_keys": table_keys,
                "subtables": subtables,
                "values": values if len(values) <= 50 else {"note": f"Too many values ({len(values)}), showing keys only", "keys": list(values.keys())[:50]}
            }
            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "list_all_topics":
            filter_str = arguments.get("filter", "")

            # Get ALL topics
            all_topics = get_all_topics()

            # Apply filter if provided
            if filter_str:
                filtered = [t for t in all_topics if filter_str.lower() in t.lower()]
            else:
                filtered = all_topics

            # Group topics by top-level table
            grouped = {}
            for topic in filtered:
                parts = topic.strip("/").split("/")
                if parts:
                    top_level = parts[0]
                    if top_level not in grouped:
                        grouped[top_level] = []
                    grouped[top_level].append(topic)

            result = {
                "total_topics": len(all_topics),
                "filtered_topics": len(filtered),
                "filter_applied": filter_str if filter_str else "none",
                "topics_by_table": {k: len(v) for k, v in grouped.items()},
                "all_topics": filtered[:100] if len(filtered) > 100 else filtered,
                "truncated": len(filtered) > 100
            }
            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        # =====================================================================
        # Simulation Control Tools (HALSim WebSocket)
        # =====================================================================

        elif name == "simulation_control_status":
            ws_available = is_simulation()
            ws_connected = _ws_connected

            result = {
                "websocket_server_available": ws_available,
                "websocket_connected": ws_connected,
                "websocket_target": f"{_ws_host}:{_ws_port}",
                "is_simulation": ws_available,
                "hint": "Simulation control tools (set_robot_mode, set_joystick_input, set_alliance) "
                        "are only available when running the WPILib simulation."
            }

            if not ws_available:
                result["note"] = (
                    "HALSim WebSocket server not detected. Make sure:\n"
                    "1. Simulation is running (./gradlew simulateJava)\n"
                    "2. build.gradle has: wpi.sim.addWebsocketsServer().defaultEnabled = true"
                )

            return [TextContent(type="text", text=json.dumps(result, indent=2))]

        elif name == "set_robot_mode":
            # Check if simulation is running
            if not is_simulation():
                return [TextContent(
                    type="text",
                    text="Simulation control not available. Make sure:\n"
                         "1. Simulation is running (./gradlew simulateJava)\n"
                         "2. build.gradle has: wpi.sim.addWebsocketsServer().defaultEnabled = true"
                )]

            enabled = arguments.get("enabled", False)
            mode = arguments.get("mode", "teleop").lower()

            autonomous = mode == "autonomous"
            test = mode == "test"

            success = await set_driver_station_mode(enabled, autonomous, test)

            if success:
                mode_str = "autonomous" if autonomous else ("test" if test else "teleop")
                state_str = "enabled" if enabled else "disabled"
                return [TextContent(
                    type="text",
                    text=f"Robot set to {mode_str} mode, {state_str}"
                )]
            else:
                return [TextContent(
                    type="text",
                    text="Failed to set robot mode. WebSocket connection error."
                )]

        elif name == "set_joystick_input":
            # Check if simulation is running
            if not is_simulation():
                return [TextContent(
                    type="text",
                    text="Simulation control not available. Make sure:\n"
                         "1. Simulation is running (./gradlew simulateJava)\n"
                         "2. build.gradle has: wpi.sim.addWebsocketsServer().defaultEnabled = true"
                )]

            port = arguments.get("joystick", 0)
            axes_obj = arguments.get("axes", {})
            buttons_obj = arguments.get("buttons", {})
            pov = arguments.get("pov", -1)

            # Convert axes object to list
            axes_list = [0.0] * 6
            if axes_obj:
                axes_list[AXIS_LEFT_X] = float(axes_obj.get("left_x", 0.0))
                axes_list[AXIS_LEFT_Y] = float(axes_obj.get("left_y", 0.0))
                axes_list[AXIS_LEFT_TRIGGER] = float(axes_obj.get("left_trigger", 0.0))
                axes_list[AXIS_RIGHT_TRIGGER] = float(axes_obj.get("right_trigger", 0.0))
                axes_list[AXIS_RIGHT_X] = float(axes_obj.get("right_x", 0.0))
                axes_list[AXIS_RIGHT_Y] = float(axes_obj.get("right_y", 0.0))

            # Convert buttons object to list (WPILib uses 1-indexed, list is 0-indexed)
            buttons_list = [False] * 12
            if buttons_obj:
                buttons_list[BUTTON_A - 1] = bool(buttons_obj.get("a", False))
                buttons_list[BUTTON_B - 1] = bool(buttons_obj.get("b", False))
                buttons_list[BUTTON_X - 1] = bool(buttons_obj.get("x", False))
                buttons_list[BUTTON_Y - 1] = bool(buttons_obj.get("y", False))
                buttons_list[BUTTON_LEFT_BUMPER - 1] = bool(buttons_obj.get("left_bumper", False))
                buttons_list[BUTTON_RIGHT_BUMPER - 1] = bool(buttons_obj.get("right_bumper", False))
                buttons_list[BUTTON_BACK - 1] = bool(buttons_obj.get("back", False))
                buttons_list[BUTTON_START - 1] = bool(buttons_obj.get("start", False))
                buttons_list[BUTTON_LEFT_STICK - 1] = bool(buttons_obj.get("left_stick", False))
                buttons_list[BUTTON_RIGHT_STICK - 1] = bool(buttons_obj.get("right_stick", False))

            # POV as list
            povs_list = [pov] if pov is not None else [-1]

            success = await set_joystick_state(
                port,
                axes=axes_list if axes_obj else None,
                buttons=buttons_list if buttons_obj else None,
                povs=povs_list if pov is not None else None
            )

            if success:
                result = {
                    "joystick": port,
                    "axes_set": bool(axes_obj),
                    "buttons_set": bool(buttons_obj),
                    "pov_set": pov is not None,
                    "status": "success"
                }
                if axes_obj:
                    result["axes"] = axes_obj
                if buttons_obj:
                    result["buttons"] = buttons_obj
                if pov is not None:
                    result["pov"] = pov
                return [TextContent(type="text", text=json.dumps(result, indent=2))]
            else:
                return [TextContent(
                    type="text",
                    text="Failed to set joystick input. WebSocket connection error."
                )]

        elif name == "set_alliance":
            # Check if simulation is running
            if not is_simulation():
                return [TextContent(
                    type="text",
                    text="Simulation control not available. Make sure:\n"
                         "1. Simulation is running (./gradlew simulateJava)\n"
                         "2. build.gradle has: wpi.sim.addWebsocketsServer().defaultEnabled = true"
                )]

            alliance = arguments.get("alliance", "blue").lower()
            station = arguments.get("station", 1)

            if alliance not in ["red", "blue"]:
                return [TextContent(
                    type="text",
                    text=f"Invalid alliance '{alliance}'. Must be 'red' or 'blue'."
                )]

            if station not in [1, 2, 3]:
                return [TextContent(
                    type="text",
                    text=f"Invalid station '{station}'. Must be 1, 2, or 3."
                )]

            success = await set_alliance_station(alliance, station)

            if success:
                return [TextContent(
                    type="text",
                    text=f"Alliance set to {alliance.capitalize()} {station}"
                )]
            else:
                return [TextContent(
                    type="text",
                    text="Failed to set alliance. WebSocket connection error."
                )]

        elif name == "drive_robot":
            # Check if simulation is running
            if not is_simulation():
                return [TextContent(
                    type="text",
                    text="Simulation control not available. Make sure:\n"
                         "1. Simulation is running (./gradlew simulateJava)\n"
                         "2. build.gradle has: wpi.sim.addWebsocketsServer().defaultEnabled = true"
                )]

            left_x = float(arguments.get("left_x", 0.0))
            left_y = float(arguments.get("left_y", 0.0))
            right_x = float(arguments.get("right_x", 0.0))
            duration = float(arguments.get("duration", 2.0))
            buttons_arg = arguments.get("buttons", {})

            # Clamp duration
            duration = max(0.1, min(10.0, duration))

            # First enable teleop mode
            if not await set_driver_station_mode(enabled=True, autonomous=False, test=False):
                return [TextContent(
                    type="text",
                    text="Failed to enable teleop mode. WebSocket connection error."
                )]

            # Build axes list
            axes_list = [left_x, left_y, 0.0, 0.0, right_x, 0.0]

            # Build buttons list (12 buttons, 1-indexed in WPILib)
            buttons_list = [False] * 12
            button_map = {
                "a": BUTTON_A - 1,
                "b": BUTTON_B - 1,
                "x": BUTTON_X - 1,
                "y": BUTTON_Y - 1,
                "left_bumper": BUTTON_LEFT_BUMPER - 1,
                "right_bumper": BUTTON_RIGHT_BUMPER - 1,
                "back": BUTTON_BACK - 1,
                "start": BUTTON_START - 1,
                "left_stick": BUTTON_LEFT_STICK - 1,
                "right_stick": BUTTON_RIGHT_STICK - 1,
            }
            held_buttons = []
            for btn_name, btn_idx in button_map.items():
                if buttons_arg.get(btn_name, False):
                    buttons_list[btn_idx] = True
                    held_buttons.append(btn_name)

            # Send joystick inputs continuously
            iterations = int(duration / 0.02)  # 20ms intervals (50Hz - match robot loop rate)
            start_time = time.time()

            for _ in range(iterations):
                # Send joystick data
                joy_success = await set_joystick_state(
                    port=0,
                    axes=axes_list,
                    buttons=buttons_list,
                    povs=[-1]
                )
                # Also re-send enabled state to ensure robot stays enabled
                mode_success = await set_driver_station_mode(enabled=True, autonomous=False, test=False)

                if not joy_success or not mode_success:
                    return [TextContent(
                        type="text",
                        text=f"WebSocket connection lost during driving after {time.time() - start_time:.1f}s"
                    )]
                await asyncio.sleep(0.02)

            # Release controls (return to neutral)
            await set_joystick_state(
                port=0,
                axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                buttons=[False] * 12,
                povs=[-1]
            )

            elapsed = time.time() - start_time
            buttons_info = f", buttons={held_buttons}" if held_buttons else ""
            return [TextContent(
                type="text",
                text=f"Drove robot for {elapsed:.1f}s with inputs: left_x={left_x}, left_y={left_y}, right_x={right_x}{buttons_info}"
            )]

        else:
            return [TextContent(type="text", text=f"Unknown tool: {name}")]

    except Exception as e:
        return [TextContent(type="text", text=f"Error: {str(e)}")]


async def main():
    parser = argparse.ArgumentParser(description="FRC NetworkTables MCP Server")
    parser.add_argument("--host", default="localhost", help="Robot host (default: localhost)")
    parser.add_argument("--port", type=int, default=5810, help="NetworkTables port (default: 5810)")
    args = parser.parse_args()

    # Connect to robot
    print(f"Connecting to robot at {args.host}:{args.port}...", file=sys.stderr)
    connect_to_robot(args.host, args.port)

    # Run MCP server
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
