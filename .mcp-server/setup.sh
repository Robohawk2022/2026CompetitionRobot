#!/bin/bash
# Setup script for FRC NetworkTables MCP Server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Setting up FRC NetworkTables MCP Server..."

# Check Python version
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is required but not installed."
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
echo "Found Python $PYTHON_VERSION"

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "Setup complete!"
echo ""
echo "The MCP server is already configured in .mcp.json at the project root."
echo "When you start Claude Code, it will prompt you to enable the frc-networktables server."
echo ""
echo "To use:"
echo "  1. Start robot simulation: ./gradlew simulateJava"
echo "  2. Start Claude Code in this project"
echo "  3. When prompted, approve the frc-networktables MCP server"
echo "  4. Ask Claude about robot state (e.g., 'What is the robot's pose?')"
echo ""
echo "To run the server manually for testing:"
echo "  source $SCRIPT_DIR/venv/bin/activate"
echo "  python $SCRIPT_DIR/frc_networktables_server.py"
