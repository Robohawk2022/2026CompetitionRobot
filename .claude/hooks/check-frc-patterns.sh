#!/bin/bash
# Checks FRC-specific code patterns from CLAUDE.md
# Non-blocking - provides warnings only

# Read JSON input from stdin
input_data=$(cat)
file_path=$(echo "$input_data" | jq -r '.tool_input.file_path // .tool_input.filePath // ""' 2>/dev/null)

# Only check Java source files
if [[ ! "$file_path" =~ \.java$ ]]; then
    exit 0
fi

# Skip non-existent files
if [[ ! -f "$file_path" ]]; then
    exit 0
fi

warnings=""

# Check for Subsystem files
if [[ "$file_path" =~ Subsystem\.java$ ]]; then

    # Check for Objects.requireNonNull in constructors
    if grep -q "public.*Subsystem(" "$file_path"; then
        if ! grep -q "Objects.requireNonNull" "$file_path"; then
            warnings+="⚠ Missing Objects.requireNonNull() for constructor parameters\n"
        fi
    fi

    # Check for voltage clamping
    if grep -q "applyVolts" "$file_path"; then
        if ! grep -q "Util.clampVolts\|clampVolts" "$file_path"; then
            warnings+="⚠ Missing Util.clampVolts() - voltages should be clamped before hardware\n"
        fi
    fi
fi

# Check for Command files
if [[ "$file_path" =~ Command\.java$ ]]; then

    # Check for addRequirements
    if grep -q "extends Command" "$file_path"; then
        if ! grep -q "addRequirements" "$file_path"; then
            warnings+="⚠ Missing addRequirements() - commands must declare subsystem dependencies\n"
        fi
    fi

    # Check for PID reset in initialize
    if grep -q "pid\|PID\|PDController" "$file_path"; then
        if grep -q "void initialize" "$file_path"; then
            if ! grep -A 10 "void initialize" "$file_path" | grep -q "reset()"; then
                warnings+="⚠ PID controller may need reset() in initialize()\n"
            fi
        fi
    fi
fi

# Check for hardcoded magic numbers in any file
if grep -E "applyVolts\([0-9]+\.[0-9]+\)|setVoltage\([0-9]+\.[0-9]+\)" "$file_path" > /dev/null 2>&1; then
    warnings+="⚠ Hardcoded voltage values detected - consider using Config.java\n"
fi

# Output warnings if any
if [[ -n "$warnings" ]]; then
    echo "FRC Pattern Checker:"
    echo -e "$warnings"
fi

exit 0  # Always non-blocking
