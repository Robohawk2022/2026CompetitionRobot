#!/bin/bash
# Helper script: Provides WPILib documentation URLs for common topics
# Usage: ./wpilib-docs.sh <topic>
# Claude can reference this to know which URLs to fetch

TOPIC="${1:-help}"

case "$TOPIC" in
    "commands"|"command")
        echo "https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html"
        ;;
    "subsystems"|"subsystem")
        echo "https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html"
        ;;
    "triggers"|"trigger"|"bindings")
        echo "https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html"
        ;;
    "pid"|"control")
        echo "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html"
        ;;
    "swerve")
        echo "https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html"
        ;;
    "odometry")
        echo "https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html"
        ;;
    "pose"|"pose-estimator"|"vision")
        echo "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html"
        ;;
    "simulation"|"sim")
        echo "https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html"
        ;;
    "motors"|"motor")
        echo "https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html"
        ;;
    "preferences")
        echo "https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html"
        ;;
    "units")
        echo "https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html"
        ;;
    "networktables"|"nt")
        echo "https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html"
        ;;
    "pathplanner"|"auto"|"autonomous")
        echo "https://pathplanner.dev/home.html"
        ;;
    "ctre"|"phoenix"|"talon"|"falcon")
        echo "https://v6.docs.ctr-electronics.com/en/stable/"
        ;;
    "rev"|"spark"|"neo")
        echo "https://docs.revrobotics.com/brushless/spark-max/overview"
        ;;
    "list"|"all")
        cat << 'EOF'
Available topics and their documentation URLs:

WPILIB CORE:
  commands     - Command-based programming
  subsystems   - Subsystem architecture
  triggers     - Binding commands to triggers
  pid          - PID controllers
  simulation   - Robot simulation
  motors       - Motor control
  preferences  - Robot preferences
  units        - WPILib units library
  networktables - NetworkTables

KINEMATICS & ODOMETRY:
  swerve       - Swerve drive kinematics
  odometry     - Swerve drive odometry
  pose         - Pose estimation & AprilTags

AUTONOMOUS:
  pathplanner  - PathPlanner autonomous paths

VENDOR LIBRARIES:
  ctre         - CTRE Phoenix 6 (TalonFX, CANcoder)
  rev          - REV Robotics (SparkMax, NEO)

Usage: ./wpilib-docs.sh <topic>
EOF
        ;;
    "help"|*)
        echo "Usage: ./wpilib-docs.sh <topic>"
        echo "Run './wpilib-docs.sh list' to see all available topics"
        ;;
esac
