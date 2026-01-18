# Session: Add FRC Command Documentation to CLAUDE.md

**Date**: 2026-01-17

## Summary

Added concise WPILib command documentation to CLAUDE.md covering command types, lifecycle, composition patterns, trigger bindings, and autonomous patterns. Then trimmed for brevity.

## Changes Made

### Files Modified
- `CLAUDE.md` - Added ~150 lines of new documentation between "Command Patterns" and "Wiring in RobotContainer" sections (initially ~490 lines, then condensed)

## New Documentation Sections

### 1. WPILib Command Types Reference
- Table of all built-in command types with factory methods
- Code examples for each type: InstantCommand, RunCommand, StartEndCommand, FunctionalCommand, WaitCommand, WaitUntilCommand, ConditionalCommand, SelectCommand, DeferredCommand

### 2. Command Lifecycle
- ASCII diagram showing scheduler loop and method execution flow
- Table explaining when each method is called (initialize, execute, isFinished, end)
- Code examples with DO/DON'T guidelines for each lifecycle method

### 3. Command Composition Patterns
- Sequential composition: `sequence()`, `andThen()`, `beforeStarting()`
- Parallel composition: `parallel()`, `race()`, `deadline()` with clear explanations of end conditions
- Decorator methods: `alongWith()`, `raceWith()`, `deadlineWith()`, `withTimeout()`, `until()`, `onlyWhile()`, `unless()`, `onlyIf()`, `finallyDo()`, `handleInterrupt()`, `repeatedly()`, `withName()`, `asProxy()`
- Summary table comparing all composition patterns

### 4. Trigger Bindings
- Table explaining all binding methods: `onTrue()`, `onFalse()`, `whileTrue()`, `whileFalse()`, `toggleOnTrue()`, `toggleOnFalse()`
- Trigger composition: `and()`, `or()`, `negate()`, `debounce()`
- Common binding patterns with realistic examples

### 5. Common Autonomous Patterns
- Simple sequential auto example
- Parallel operations with deadline pattern
- Conditional auto using `Commands.either()`
- Auto with timeout safety and cleanup using `finallyDo()`

## Testing Done

- [x] Verified markdown formatting renders correctly
- [x] Code examples follow project patterns (command factories, Config usage)
- [ ] `./gradlew build` - N/A (documentation only)

## Notes for Next Session

The documentation is comprehensive but could be expanded in the future with:
- PathPlanner/Choreo integration patterns for auto
- Swerve-specific command patterns
- Vision-guided command examples
