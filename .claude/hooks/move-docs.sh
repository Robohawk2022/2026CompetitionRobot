#!/bin/bash
# Hook: PostToolUse (Write|Edit) - Moves files from docs/ to .claude-docs/
# This keeps session logs in a hidden folder while allowing Claude to write to docs/

# Read the JSON input from stdin
INPUT=$(cat)

# Extract the file path from the tool input
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // ""' 2>/dev/null)

# Check if the file is in the docs/ folder
if [[ "$FILE_PATH" == *"/docs/"* ]]; then
    # Get the filename
    FILENAME=$(basename "$FILE_PATH")

    # Build the destination path
    PROJECT_DIR=$(dirname "$FILE_PATH" | sed 's|/docs$||' | sed 's|/docs/.*||')
    # Find the project root by looking for .claude-docs
    DEST_DIR="$CLAUDE_PROJECT_DIR/.claude-docs"
    DEST_PATH="$DEST_DIR/$FILENAME"

    # Create destination directory if it doesn't exist
    mkdir -p "$DEST_DIR"

    # Move the file if it exists
    if [[ -f "$FILE_PATH" ]]; then
        mv "$FILE_PATH" "$DEST_PATH"

        # Remove the docs folder if empty
        DOCS_DIR=$(dirname "$FILE_PATH")
        rmdir "$DOCS_DIR" 2>/dev/null

        # Output message to Claude
        cat << EOF
{
  "hookSpecificOutput": {
    "hookEventName": "PostToolUse",
    "additionalContext": "File automatically moved from docs/ to .claude-docs/: $FILENAME"
  }
}
EOF
    fi
fi

exit 0
