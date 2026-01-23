#!/usr/bin/env python3
"""
FRC Documentation MCP Server

Provides tools for fetching WPILib documentation, searching Chief Delphi,
and accessing other FRC-related resources.

Tools:
- wpilib_docs: Fetch WPILib documentation by topic
- chief_delphi_search: Search Chief Delphi forum
- ctre_docs: Fetch CTRE Phoenix 6 documentation
- rev_docs: Fetch REV Robotics documentation
- pathplanner_docs: Fetch PathPlanner documentation
"""

import json
import sys
import urllib.request
import urllib.parse
import urllib.error
import ssl
from typing import Any

# MCP Protocol version
PROTOCOL_VERSION = "2024-11-05"

# Documentation URL mappings
WPILIB_DOCS = {
    "commands": "https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html",
    "subsystems": "https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html",
    "triggers": "https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html",
    "pid": "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html",
    "feedforward": "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html",
    "swerve": "https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html",
    "odometry": "https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html",
    "pose-estimator": "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html",
    "apriltag": "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html",
    "simulation": "https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html",
    "motors": "https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html",
    "preferences": "https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html",
    "units": "https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html",
    "networktables": "https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html",
    "shuffleboard": "https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html",
    "command-groups": "https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html",
    "scheduling": "https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html",
}

CTRE_DOCS = {
    "overview": "https://v6.docs.ctr-electronics.com/en/stable/",
    "talonfx": "https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/index.html",
    "cancoder": "https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html",
    "pigeon2": "https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/pigeon2/index.html",
    "swerve": "https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html",
    "motion-magic": "https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html",
    "current-limits": "https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/current-limits.html",
}


def send_response(response: dict) -> None:
    """Send a JSON-RPC response to stdout."""
    json.dump(response, sys.stdout)
    sys.stdout.write("\n")
    sys.stdout.flush()


def send_error(id: Any, code: int, message: str) -> None:
    """Send a JSON-RPC error response."""
    send_response({
        "jsonrpc": "2.0",
        "id": id,
        "error": {"code": code, "message": message}
    })


def fetch_url(url: str) -> str:
    """Fetch content from a URL."""
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    req = urllib.request.Request(
        url,
        headers={"User-Agent": "FRC-MCP-Server/1.0"}
    )

    try:
        with urllib.request.urlopen(req, context=ctx, timeout=30) as response:
            return response.read().decode("utf-8")
    except urllib.error.HTTPError as e:
        return f"HTTP Error {e.code}: {e.reason}"
    except urllib.error.URLError as e:
        return f"URL Error: {e.reason}"
    except Exception as e:
        return f"Error: {str(e)}"


def handle_initialize(id: Any, params: dict) -> None:
    """Handle the initialize request."""
    send_response({
        "jsonrpc": "2.0",
        "id": id,
        "result": {
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {
                "tools": {}
            },
            "serverInfo": {
                "name": "frc-docs-server",
                "version": "1.0.0"
            }
        }
    })


def handle_tools_list(id: Any) -> None:
    """Handle the tools/list request."""
    tools = [
        {
            "name": "wpilib_docs",
            "description": "Fetch WPILib documentation for a specific topic. Available topics: " + ", ".join(WPILIB_DOCS.keys()),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {
                        "type": "string",
                        "description": "The documentation topic to fetch",
                        "enum": list(WPILIB_DOCS.keys())
                    }
                },
                "required": ["topic"]
            }
        },
        {
            "name": "wpilib_docs_url",
            "description": "Get the URL for a WPILib documentation topic without fetching content",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {
                        "type": "string",
                        "description": "The documentation topic",
                        "enum": list(WPILIB_DOCS.keys())
                    }
                },
                "required": ["topic"]
            }
        },
        {
            "name": "chief_delphi_search",
            "description": "Generate a Chief Delphi search URL for FRC community discussions. Returns a search URL that can be used with WebFetch or WebSearch.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "Search query (e.g., 'swerve drive code java', 'phoenix 6 current limits')"
                    },
                    "category": {
                        "type": "string",
                        "description": "Optional category filter",
                        "enum": ["all", "programming", "technical", "build"]
                    }
                },
                "required": ["query"]
            }
        },
        {
            "name": "ctre_docs",
            "description": "Fetch CTRE Phoenix 6 documentation. Available topics: " + ", ".join(CTRE_DOCS.keys()),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {
                        "type": "string",
                        "description": "The CTRE documentation topic",
                        "enum": list(CTRE_DOCS.keys())
                    }
                },
                "required": ["topic"]
            }
        },
        {
            "name": "ctre_docs_url",
            "description": "Get the URL for a CTRE documentation topic without fetching content",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {
                        "type": "string",
                        "description": "The CTRE documentation topic",
                        "enum": list(CTRE_DOCS.keys())
                    }
                },
                "required": ["topic"]
            }
        },
        {
            "name": "list_doc_topics",
            "description": "List all available documentation topics for WPILib and CTRE",
            "inputSchema": {
                "type": "object",
                "properties": {}
            }
        },
        {
            "name": "frc_resources",
            "description": "Get URLs for common FRC programming resources",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "resource": {
                        "type": "string",
                        "description": "The resource to get",
                        "enum": ["wpilib", "ctre", "rev", "pathplanner", "photonvision", "limelight", "chief-delphi", "frc-docs"]
                    }
                },
                "required": ["resource"]
            }
        }
    ]

    send_response({
        "jsonrpc": "2.0",
        "id": id,
        "result": {"tools": tools}
    })


def handle_tools_call(id: Any, params: dict) -> None:
    """Handle the tools/call request."""
    tool_name = params.get("name", "")
    arguments = params.get("arguments", {})

    result_text = ""

    if tool_name == "wpilib_docs":
        topic = arguments.get("topic", "")
        if topic in WPILIB_DOCS:
            url = WPILIB_DOCS[topic]
            result_text = f"Fetching WPILib docs for '{topic}'...\n\nURL: {url}\n\nUse WebFetch to get the full content from this URL."
        else:
            result_text = f"Unknown topic: {topic}. Available: {', '.join(WPILIB_DOCS.keys())}"

    elif tool_name == "wpilib_docs_url":
        topic = arguments.get("topic", "")
        if topic in WPILIB_DOCS:
            result_text = WPILIB_DOCS[topic]
        else:
            result_text = f"Unknown topic: {topic}. Available: {', '.join(WPILIB_DOCS.keys())}"

    elif tool_name == "chief_delphi_search":
        query = arguments.get("query", "")
        category = arguments.get("category", "all")

        encoded_query = urllib.parse.quote(query)

        # Chief Delphi category IDs
        category_ids = {
            "programming": "c/first-tech-challenge/programming/51",
            "technical": "c/technical/8",
            "build": "c/technical/build/20"
        }

        if category != "all" and category in category_ids:
            search_url = f"https://www.chiefdelphi.com/search?q={encoded_query}%20%23{category_ids[category]}"
        else:
            search_url = f"https://www.chiefdelphi.com/search?q={encoded_query}"

        result_text = f"""Chief Delphi Search Results for: "{query}"

Search URL: {search_url}

You can also use WebSearch with: site:chiefdelphi.com {query}

Tips for better results:
- Add "java" for Java-specific code
- Add "2024" or "2025" for recent discussions
- Common topics: swerve, vision, autonomous, TalonFX, SparkMax"""

    elif tool_name == "ctre_docs":
        topic = arguments.get("topic", "")
        if topic in CTRE_DOCS:
            url = CTRE_DOCS[topic]
            result_text = f"Fetching CTRE docs for '{topic}'...\n\nURL: {url}\n\nUse WebFetch to get the full content from this URL."
        else:
            result_text = f"Unknown topic: {topic}. Available: {', '.join(CTRE_DOCS.keys())}"

    elif tool_name == "ctre_docs_url":
        topic = arguments.get("topic", "")
        if topic in CTRE_DOCS:
            result_text = CTRE_DOCS[topic]
        else:
            result_text = f"Unknown topic: {topic}. Available: {', '.join(CTRE_DOCS.keys())}"

    elif tool_name == "list_doc_topics":
        result_text = """Available Documentation Topics:

=== WPILib Topics ===
""" + "\n".join(f"  - {k}: {v}" for k, v in WPILIB_DOCS.items()) + """

=== CTRE Phoenix 6 Topics ===
""" + "\n".join(f"  - {k}: {v}" for k, v in CTRE_DOCS.items())

    elif tool_name == "frc_resources":
        resource = arguments.get("resource", "")
        resources = {
            "wpilib": "https://docs.wpilib.org/en/stable/",
            "ctre": "https://v6.docs.ctr-electronics.com/en/stable/",
            "rev": "https://docs.revrobotics.com/",
            "pathplanner": "https://pathplanner.dev/home.html",
            "photonvision": "https://docs.photonvision.org/en/latest/",
            "limelight": "https://docs.limelightvision.io/",
            "chief-delphi": "https://www.chiefdelphi.com/",
            "frc-docs": "https://docs.wpilib.org/en/stable/"
        }

        if resource in resources:
            result_text = resources[resource]
        else:
            result_text = "Available resources:\n" + "\n".join(f"  - {k}: {v}" for k, v in resources.items())

    else:
        send_error(id, -32601, f"Unknown tool: {tool_name}")
        return

    send_response({
        "jsonrpc": "2.0",
        "id": id,
        "result": {
            "content": [{"type": "text", "text": result_text}]
        }
    })


def main():
    """Main loop - read JSON-RPC messages from stdin and respond."""
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            request = json.loads(line)
        except json.JSONDecodeError as e:
            send_error(None, -32700, f"Parse error: {e}")
            continue

        request_id = request.get("id")
        method = request.get("method", "")
        params = request.get("params", {})

        if method == "initialize":
            handle_initialize(request_id, params)
        elif method == "notifications/initialized":
            # Client notification, no response needed
            pass
        elif method == "tools/list":
            handle_tools_list(request_id)
        elif method == "tools/call":
            handle_tools_call(request_id, params)
        elif method == "ping":
            send_response({"jsonrpc": "2.0", "id": request_id, "result": {}})
        else:
            # Unknown method - send empty result for notifications, error for requests
            if request_id is not None:
                send_error(request_id, -32601, f"Method not found: {method}")


if __name__ == "__main__":
    main()
