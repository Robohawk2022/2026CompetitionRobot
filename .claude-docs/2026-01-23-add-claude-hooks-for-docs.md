# Session: Add Claude Code Hooks and MCP Servers for Documentation

**Date**: 2026-01-23

## Summary

Created Claude Code hooks to remind Claude to check WPILib docs and Chief Delphi before implementing FRC code. Added MCP server for FRC documentation lookup. Added helper scripts for documentation URLs and expanded WebFetch permissions.

## Changes Made

### Files Created
- `.claude/hooks/check-docs-reminder.sh` - UserPromptSubmit hook that adds a reminder to check WPILib docs, CTRE docs, Chief Delphi, and claude-docs/ before implementing code
- `.claude/hooks/wpilib-docs.sh` - Helper script providing documentation URLs for common WPILib topics (commands, subsystems, swerve, PID, etc.)
- `.claude/hooks/chief-delphi.sh` - Helper script for generating Chief Delphi search URLs
- `mcp-server/frc_docs_server.py` - MCP server providing tools for fetching WPILib, CTRE, and searching Chief Delphi

### Files Modified
- `.claude/settings.local.json` - Added UserPromptSubmit hook configuration and new WebFetch domain permissions
- `.mcp.json` - Added frc-docs MCP server configuration

## MCP Servers Available

### frc-networktables (existing)
Connects to robot NetworkTables for live data. Tools:
- `get_robot_state` - Enabled/disabled, mode, alliance
- `get_robot_pose` - Position and heading
- `get_swerve_status` - Velocities, module states
- `get_vision_status` - Tag detection, pose estimates
- `get_active_commands` - Running commands
- `get_odometry_diagnostics` - Drift, vision accept rate
- `get_subsystem_status` - Query any subsystem
- `get_preference` / `list_preferences` - Config values
- `get_networktables_value` / `list_networktables` - Raw NT access

### frc-docs (new)
Provides documentation URLs and search. Tools:
- `wpilib_docs` - Fetch WPILib doc topic (commands, subsystems, swerve, pid, etc.)
- `wpilib_docs_url` - Get URL without fetching
- `chief_delphi_search` - Generate Chief Delphi search URL
- `ctre_docs` / `ctre_docs_url` - CTRE Phoenix 6 documentation
- `list_doc_topics` - List all available topics
- `frc_resources` - Get URLs for common FRC resources

## How the Hooks Work

### UserPromptSubmit Hook
The `check-docs-reminder.sh` hook fires when the user submits a prompt. It:
1. Reads the user's prompt from stdin (JSON format)
2. Checks if the prompt contains implementation-related keywords (implement, create, add, build, etc.)
3. If matched, adds context reminding Claude to check:
   - WPILib documentation (docs.wpilib.org)
   - CTRE Phoenix 6 docs (v6.docs.ctr-electronics.com)
   - Chief Delphi (via WebSearch with site:chiefdelphi.com)
   - Previous session logs in claude-docs/

### Helper Scripts

**wpilib-docs.sh** - Returns documentation URLs for topics:
```bash
./wpilib-docs.sh commands    # -> Command-based docs URL
./wpilib-docs.sh swerve      # -> Swerve kinematics docs URL
./wpilib-docs.sh ctre        # -> CTRE Phoenix 6 docs URL
./wpilib-docs.sh list        # -> Shows all available topics
```

Available topics: commands, subsystems, triggers, pid, swerve, odometry, pose, simulation, motors, preferences, units, networktables, pathplanner, ctre, rev

**chief-delphi.sh** - Generates Chief Delphi search URLs:
```bash
./chief-delphi.sh "swerve module code java"
# -> https://www.chiefdelphi.com/search?q=swerve%20module%20code%20java
```

## New Permissions Added

| Domain | Purpose |
|--------|---------|
| `www.chiefdelphi.com` | Chief Delphi forum |
| `chiefdelphi.com` | Chief Delphi forum (alternate) |
| `pathplanner.dev` | PathPlanner autonomous documentation |
| `docs.revrobotics.com` | REV Robotics documentation |

## Usage Examples

### Using MCP Tools
```
# Get WPILib doc URL
mcp__frc_docs__wpilib_docs_url topic=commands

# Search Chief Delphi
mcp__frc_docs__chief_delphi_search query="swerve pid tuning java"

# List all doc topics
mcp__frc_docs__list_doc_topics

# Get robot pose (when simulation running)
mcp__frc_networktables__get_robot_pose
```

### Searching Chief Delphi
Claude can search Chief Delphi using WebSearch:
```
WebSearch: site:chiefdelphi.com phoenix 6 swerve java
```

Or fetch specific threads:
```
WebFetch: https://www.chiefdelphi.com/t/some-thread-id
```

### Fetching WPILib Docs
```
WebFetch: https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
```

### Automatic Reminders
When a user asks to implement something (e.g., "implement a shooter subsystem"), the hook automatically adds context reminding Claude to check relevant documentation first.

## Testing Done

- [x] Scripts are executable (chmod +x)
- [x] settings.local.json is valid JSON
- [x] .mcp.json is valid JSON
- [ ] Hook tested in new Claude Code session (requires session restart)
- [ ] MCP servers tested (requires session restart)

## Known Limitations

- Hooks only provide reminders/context - they don't force Claude to actually fetch docs
- The keyword matching in check-docs-reminder.sh is simple grep-based
- Hooks require `jq` to be installed for JSON parsing
- MCP servers require Python 3.10+ and dependencies from mcp-server/requirements.txt
- NetworkTables MCP only works when simulation is running

## Setup Required

To use the MCP servers:
```bash
cd mcp-server
chmod +x setup.sh
./setup.sh
```

This installs pyntcore and mcp Python packages.

## Notes for Next Session

- The hooks and MCP servers will take effect in the next Claude Code session
- To test, start a new session and ask to implement something
- You should see Claude reference the documentation reminder in its thinking
- MCP tools will appear as `/mcp__frc_docs__*` and `/mcp__frc_networktables__*`
- If hooks don't work, check that the scripts are executable and that the JSON is valid
