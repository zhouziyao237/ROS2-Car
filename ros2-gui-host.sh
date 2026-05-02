#!/bin/bash
# Filename: ros2-gui-host.sh
# Purpose: Fix X11 forwarding on the host and launch a ROS2 GUI application.
# Usage: ./ros2-gui-host.sh [command]
# Example: ./ros2-gui-host.sh rviz2
#          ./ros2-gui-host.sh gazebo
#          ./ros2-gui-host.sh bash

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY is not set. Are you connected with SSH X11 forwarding (-X or -Y)?"
    exit 1
fi

# Extract display number (e.g., from localhost:10.0 -> 10, or ros2-server:11.0 -> 11)
DISPLAY_NUM=$(echo "$DISPLAY" | sed -n 's/.*:\([0-9]\+\)\..*/\1/p')
if [ -z "$DISPLAY_NUM" ]; then
    DISPLAY_NUM=$(echo "$DISPLAY" | sed -n 's/^:\([0-9]\+\)$/\1/p')
fi
if [ -z "$DISPLAY_NUM" ]; then
    echo "Error: Cannot extract display number from DISPLAY='$DISPLAY'"
    exit 1
fi

echo "Detected DISPLAY=$DISPLAY (display number :$DISPLAY_NUM)"

# Ensure xauth is installed
if ! command -v xauth &> /dev/null; then
    echo "xauth not found. Installing..."
    sudo apt update && sudo apt install -y xauth
fi

# Get the current xauth cookie for this display
# Try exact match first
COOKIE_ENTRY=$(xauth list "$DISPLAY" 2>/dev/null | head -1)
if [ -z "$COOKIE_ENTRY" ]; then
    # Fallback: match by display number only (ignore hostname)
    COOKIE_ENTRY=$(xauth list 2>/dev/null | grep ":$DISPLAY_NUM " | head -1)
fi
if [ -z "$COOKIE_ENTRY" ]; then
    echo "Error: No xauth entry found for DISPLAY=$DISPLAY (display number :$DISPLAY_NUM)"
    echo "Please run 'xauth list' on the host to verify."
    exit 1
fi

# Extract cookie value (third field)
COOKIE_VALUE=$(echo "$COOKIE_ENTRY" | awk '{print $3}')
echo "Found cookie for display $DISPLAY"

# Create a session-specific .Xauthority file in /tmp
SESSION_XAUTH="/tmp/.Xauthority_$$"
touch "$SESSION_XAUTH"
chmod 600 "$SESSION_XAUTH"

# === FIX: Use "add" with separate arguments instead of passing the whole entry ===
# Add entry for the original display (e.g., ros2-server:11)
xauth -f "$SESSION_XAUTH" add "$DISPLAY" . "$COOKIE_VALUE"
# Add entry for localhost with same cookie
xauth -f "$SESSION_XAUTH" add "localhost:$DISPLAY_NUM" . "$COOKIE_VALUE"

# Export the new XAUTHORITY and set DISPLAY to localhost (more compatible)
export XAUTHORITY="$SESSION_XAUTH"
export DISPLAY="localhost:$DISPLAY_NUM"

echo "Session XAUTHORITY set to $SESSION_XAUTH"
echo "DISPLAY set to $DISPLAY"
echo "Current cookies in session:"
xauth -f "$XAUTHORITY" list

# Determine the command to run
if [ $# -eq 0 ]; then
    echo "No command provided. Starting an interactive bash shell with X11 forwarding ready."
    CMD="bash"
else
    CMD="$@"
fi

echo "Running: $CMD"
exec $CMD