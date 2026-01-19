# Claude Session Logs

This folder contains logs of AI coding assistant sessions. Each file documents what was done during a session, including:

- Files created, modified, or deleted
- Config changes
- Commands added
- Testing performed
- Known issues and TODOs

## Purpose

These logs help:
1. **Track changes** - Know what was modified and why
2. **Onboard new sessions** - AI assistants can read past logs for context
3. **Debug issues** - Trace when and why changes were made
4. **Team communication** - Human developers can see what AI assistants did

## File Naming

Files are named: `YYYY-MM-DD-brief-description.md`

Examples:
- `2025-01-15-add-shooter-subsystem.md`
- `2025-01-15-fix-arm-pid-tuning.md`
- `2025-01-16-refactor-swerve-commands.md`

## Rules

- **Never delete** existing log files
- **Always create** a new log at the end of each session
- **Be specific** about what changed and why
- **Include testing results** so issues can be traced
