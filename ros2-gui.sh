#!/bin/bash
# Filename: ros2-gui.sh
# Purpose: Manage a persistent ROS2 GUI container with dynamic X11 cookie injection.
#          Creates a session-specific writable .Xauthority file inside container,
#          completely bypassing mounted file permission issues.

# ========== Configuration (Modify these to match your setup) ==========
CONTAINER_NAME="my-ros2"           # Container name
IMAGE_NAME="my-ros2-env"                # Your Docker image name
WORKSPACE_VOLUME="ros2_workspace"   # Docker volume for persistent workspace
HOST_MOUNT_DIR="/home/ros2-server/ros2-car"   # Host directory to mount into container
# ======================================================================

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: docker command not found. Please install Docker first."
    exit 1
fi

# Allow Docker containers to connect to the X server (needs to be done only once)
xhost +local:docker &> /dev/null

# Display help
show_help() {
    cat << EOF
Usage: $0 {start|exec|stop|rm} [extra docker run options...]

Commands:
  start   - Create and start the persistent container. You can append extra
            docker run options, e.g.:
              $0 start --device /dev/bus/usb:/dev/bus/usb
              $0 start -v /extra/data:/data --env ROS_DOMAIN_ID=42
  exec    - Enter the container with dynamic X11 cookie injection
  stop    - Stop the container
  rm      - Remove the container

Description:
  - Uses host networking (--net=host) so container shares host's network.
  - Mounts host's .Xauthority (read-only) and X11 socket as fallback.
  - On 'exec', creates a session-specific temporary .Xauthority file inside container.
  - Injects current X11 cookie for both original hostname and localhost.
  - Sets DISPLAY=":$DISPLAY_NUM" for maximum compatibility.
  - Each session gets its own authentication file, avoiding permission issues.
EOF
}

# Start the container (create if not exists, start if stopped)
# Modified to accept extra docker run arguments
start_container() {
    local extra_args=("$@")   # Capture all additional arguments
    if docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "Container $CONTAINER_NAME already exists."
        if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
            echo "Starting stopped container $CONTAINER_NAME..."
            docker start "$CONTAINER_NAME"
        else
            echo "Container $CONTAINER_NAME is already running."
        fi
    else
        echo "Creating and starting new container $CONTAINER_NAME ..."
        # Create volume if it doesn't exist
        docker volume inspect "$WORKSPACE_VOLUME" &> /dev/null || docker volume create "$WORKSPACE_VOLUME"
        # Run container in background (keeps running), inserting extra args before image name
        docker run -it -d \
            --name "$CONTAINER_NAME" \
            --net=host \
            -v "$WORKSPACE_VOLUME:/root/workspace" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
            -v "$HOST_MOUNT_DIR:/host-home" \
            "${extra_args[@]}" \          # <-- Extra arguments inserted here
            "$IMAGE_NAME" \
            bash -c "while true; do sleep 3600; done"
        echo "Container started."

        echo "Please ensure that 'xauth' is installed inside the container."
        echo "If not, run: apt update && apt install -y xauth"
    fi
}

# Enter the container with dynamic X11 cookie injection
exec_container() {
    if ! docker inspect "$CONTAINER_NAME" &> /dev/null; then
        echo "Container $CONTAINER_NAME does not exist. Run '$0 start' first."
        exit 1
    fi
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]; then
        echo "Container $CONTAINER_NAME is not running. Starting it..."
        docker start "$CONTAINER_NAME"
    fi

    # Step 1: Run auto-fix-x11.sh on host if available (optional, helps but not required)
    if [ -f "$HOME/ros2-car/scripts/auto-fix-x11.sh" ]; then
        echo "Updating host X11 authentication with auto-fix-x11.sh..."
        "$HOME/ros2-car/scripts/auto-fix-x11.sh"
    else
        echo "Note: $HOME/ros2-car/scripts/auto-fix-x11.sh not found. Continuing with direct injection."
    fi

    # Step 2: Extract display number from current DISPLAY (e.g., from ros2-server:10.0 -> 10)
    DISPLAY_NUM=$(echo "$DISPLAY" | sed -n 's/.*:\([0-9]*\)\..*/\1/p')
    if [ -z "$DISPLAY_NUM" ]; then
        echo "Error: Cannot extract display number from DISPLAY='$DISPLAY'"
        exit 1
    fi

    # Step 3: Retrieve the full xauth cookie entry for current DISPLAY from host
    # First try exact match with current DISPLAY
    COOKIE_ENTRY=$(xauth list "$DISPLAY" 2>/dev/null | head -1)
    if [ -z "$COOKIE_ENTRY" ]; then
        # Fallback: try to match by display number (ignore hostname)
        COOKIE_ENTRY=$(xauth list 2>/dev/null | grep ":$DISPLAY_NUM" | grep -v unix | head -1)
    fi
    if [ -z "$COOKIE_ENTRY" ]; then
        echo "Error: No xauth entry found for DISPLAY=$DISPLAY (display number :$DISPLAY_NUM)"
        echo "Please run 'xauth list' on the host to verify."
        exit 1
    fi

    # Extract the cookie value (third field)
    COOKIE_VALUE=$(echo "$COOKIE_ENTRY" | awk '{print $3}')

    echo "Injecting X11 cookie for display :$DISPLAY_NUM into container..."

    # Step 4: Construct the command to run inside container
    INNER_CMD="
        # Create session-specific temporary file using PID for uniqueness
        SESSION_XAUTH=/tmp/.Xauthority_\$\$
        touch \"\$SESSION_XAUTH\"
        chmod 600 \"\$SESSION_XAUTH\"

        # Inject the original hostname entry (e.g., ros2-server:11)
        echo 'Adding entry: $COOKIE_ENTRY'
        xauth -f \"\$SESSION_XAUTH\" add $COOKIE_ENTRY

        # Inject localhost entry with the same cookie value
        echo \"Adding entry: localhost:$DISPLAY_NUM MIT-MAGIC-COOKIE-1 $COOKIE_VALUE\"
        xauth -f \"\$SESSION_XAUTH\" add localhost:$DISPLAY_NUM MIT-MAGIC-COOKIE-1 $COOKIE_VALUE

        # Set XAUTHORITY to the session-specific file
        export XAUTHORITY=\"\$SESSION_XAUTH\"

        echo ''
        echo 'Session-specific X11 authentication cookies (from \$XAUTHORITY):'
        xauth -f \"\$XAUTHORITY\" list
        echo ''

        # Start interactive shell
        exec bash
    "

    docker exec -it \
        -e DISPLAY=":$DISPLAY_NUM" \
        "$CONTAINER_NAME" \
        bash -c "$INNER_CMD"
}

# Stop the container
stop_container() {
    echo "Stopping container $CONTAINER_NAME ..."
    docker stop "$CONTAINER_NAME"
}

# Remove the container
rm_container() {
    echo "Removing container $CONTAINER_NAME ..."
    docker rm -f "$CONTAINER_NAME"
}

# Main command parsing
case "${1:-}" in
    start)
        shift          # Remove 'start' from argument list
        start_container "$@"   # Pass remaining arguments to function
        ;;
    exec|enter)
        exec_container
        ;;
    stop)
        stop_container
        ;;
    rm|remove)
        rm_container
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo "Error: unknown command '$1'"
        show_help
        exit 1
        ;;
esac