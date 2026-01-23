#!/bin/bash
# Hook: UserPromptSubmit - Reminds Claude to check WPILib docs before implementing FRC code
# This hook adds context to Claude's thinking when the user's prompt involves coding tasks

# Read the JSON input from stdin
INPUT=$(cat)

# Extract the user's prompt
PROMPT=$(echo "$INPUT" | jq -r '.user_prompt // ""' 2>/dev/null)

# Check if the prompt contains implementation-related keywords
if echo "$PROMPT" | grep -qiE "(implement|create|add|build|make|write|code|fix|update|modify|change|new subsystem|new command|feature)"; then
    # Output additional context for Claude
    cat << 'EOF'
{
  "hookSpecificOutput": {
    "hookEventName": "UserPromptSubmit",
    "additionalContext": "REMINDER: Before implementing FRC robot code, consider checking relevant documentation:\n\n1. **WPILib Docs** - Use WebFetch with docs.wpilib.org for official WPILib documentation\n   - Command-based: https://docs.wpilib.org/en/stable/docs/software/commandbased/\n   - Subsystems: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html\n   - Commands: https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html\n\n2. **CTRE Phoenix 6** - Use WebFetch with v6.docs.ctr-electronics.com or api.ctr-electronics.com\n\n3. **Chief Delphi** - Use WebSearch to search chiefdelphi.com for community solutions and discussions\n\n4. **Check claude-docs/** - Read previous session logs for context on what's already been implemented"
  }
}
EOF
fi

exit 0
