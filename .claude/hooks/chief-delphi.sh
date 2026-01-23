#!/bin/bash
# Helper script: Provides Chief Delphi search URLs for FRC topics
# Usage: ./chief-delphi.sh "<search query>"
# Claude can use WebSearch with site:chiefdelphi.com or WebFetch these URLs

QUERY="${1:-}"

if [ -z "$QUERY" ]; then
    cat << 'EOF'
Chief Delphi Search Helper
==========================

Usage: ./chief-delphi.sh "<search query>"

This generates a search URL for Chief Delphi (chiefdelphi.com),
the premier FRC community forum.

Claude can search Chief Delphi in two ways:

1. WebSearch with site filter:
   Use WebSearch with query: "site:chiefdelphi.com <your search terms>"

2. Direct URL:
   https://www.chiefdelphi.com/search?q=<encoded-query>

Common search topics for FRC programming:
-----------------------------------------
  "swerve drive code"
  "phoenix 6 swerve"
  "pathplanner autonomous"
  "limelight pose estimation"
  "command-based subsystem"
  "TalonFX current limiting"
  "REV SparkMax PID"
  "april tag localization"
  "odometry drift"
  "robot characterization"

Example searches:
  ./chief-delphi.sh "swerve module code java"
  ./chief-delphi.sh "phoenix 6 CANcoder"
  ./chief-delphi.sh "limelight megatag2"

Tips for searching:
  - Include "java" for Java-specific code
  - Include "2024" or "2025" for recent discussions
  - Include "code" for programming-related threads
  - Common tags: #programming, #swerve, #vision, #autonomous
EOF
    exit 0
fi

# URL encode the query
ENCODED_QUERY=$(echo -n "$QUERY" | jq -sRr @uri)

# Output the search URL
echo "https://www.chiefdelphi.com/search?q=${ENCODED_QUERY}"

# Also output the WebSearch command suggestion
echo ""
echo "Or use WebSearch with: site:chiefdelphi.com ${QUERY}"
