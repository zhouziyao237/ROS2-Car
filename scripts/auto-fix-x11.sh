#!/bin/bash
# auto-fix-x11.sh - Automatically add localhost/127.0.0.1 xauth entries for current DISPLAY

# Only run if connected via SSH and DISPLAY is set
if [ -z "$SSH_CONNECTION" ] || [ -z "$DISPLAY" ]; then
    exit 0
fi

# Extract display number (e.g., from localhost:10.0 -> 10)
disp_num=$(echo "$DISPLAY" | sed -n 's/.*:\([0-9]*\)\..*/\1/p')
if [ -z "$disp_num" ]; then
    echo "$(date): auto-fix-x11: Failed to extract display number from DISPLAY='$DISPLAY'" >> ~/x11-fix.log
    exit 1
fi

# Get the cookie from an existing entry for this display number
# Use the exact $DISPLAY to get current cookie, fallback to grep
cookie=$(xauth list "$DISPLAY" 2>/dev/null | awk '{print $3}')
if [ -z "$cookie" ]; then
    cookie=$(xauth list 2>/dev/null | grep ":$disp_num" | grep -v "unix" | head -1 | awk '{print $3}')
fi

if [ -z "$cookie" ]; then
    echo "$(date): auto-fix-x11: No cookie found for display :$disp_num" >> ~/x11-fix.log
    exit 1
fi

# Add entries for localhost and 127.0.0.1 if missing
added=0
if ! xauth list "localhost:$disp_num" 2>/dev/null | grep -q "$cookie"; then
    xauth add "localhost:$disp_num" MIT-MAGIC-COOKIE-1 "$cookie" 2>/dev/null && added=1
fi
if ! xauth list "127.0.0.1:$disp_num" 2>/dev/null | grep -q "$cookie"; then
    xauth add "127.0.0.1:$disp_num" MIT-MAGIC-COOKIE-1 "$cookie" 2>/dev/null && added=1
fi

if [ $added -eq 1 ]; then
    echo "$(date): auto-fix-x11: Added localhost/127.0.0.1 entries for :$disp_num" >> ~/x11-fix.log
fi