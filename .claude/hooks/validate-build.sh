#!/bin/bash
# Validates build after Java file edits
# Exit 0 = success (with output), Exit 1 = warning (non-blocking)

# Read JSON input from stdin
input_data=$(cat)
file_path=$(echo "$input_data" | jq -r '.tool_input.file_path // .tool_input.filePath // ""' 2>/dev/null)

# Only validate Java source files
if [[ ! "$file_path" =~ \.java$ ]]; then
    exit 0
fi

cd "$CLAUDE_PROJECT_DIR" || exit 0

echo "Running build validation..."

# Run Gradle compile (fast check)
output=$(./gradlew compileJava --quiet 2>&1)
exit_code=$?

if [[ $exit_code -eq 0 ]]; then
    echo "✓ Build passed"
    exit 0
else
    echo "⚠ BUILD FAILED - Errors detected:"
    echo "$output"
    echo ""
    echo "Please fix the compile errors above."
    exit 1  # Non-blocking warning
fi
