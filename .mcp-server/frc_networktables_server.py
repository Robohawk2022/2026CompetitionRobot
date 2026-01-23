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
import json
import socket
import sys
import time
from typing import Any

import ntcore
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool, TextContent

# Global NetworkTables instance
_nt_inst: ntcore.NetworkTableInstance | None = None
_connected = False
_current_host = "localhost"
_current_port = 5810
_multi_subscriber = None  # Subscribe to all topics


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
        )
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict[str, Any]) -> list[TextContent]:
    """Handle tool calls."""
    nt = get_nt()

    # Check connection (allow some tools to work without connection)
    connection_optional_tools = ["connection_status", "scan_for_servers", "reconnect"]
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
