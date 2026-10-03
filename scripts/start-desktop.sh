#!/bin/bash
# Start the container's virtual desktop: Xvfb, Openbox, x11vnc and noVNC.
# Idempotent: services that are already running are left alone.
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
SCREEN="${SCREEN_GEOMETRY:-1600x1000x24}"
display_num="${DISPLAY#:}"

# 1. Virtual X server with GLX, rendered in software by Mesa.
if ! pgrep -x Xvfb > /dev/null; then
    echo "Starting Xvfb on DISPLAY=${DISPLAY} (${SCREEN})"
    Xvfb "${DISPLAY}" -screen 0 "${SCREEN}" -ac +extension GLX +render -noreset \
        > /tmp/xvfb.log 2>&1 &
fi

# Wait for the X socket instead of sleeping a fixed time.
for _ in $(seq 1 50); do
    [ -S "/tmp/.X11-unix/X${display_num}" ] && break
    sleep 0.1
done
if [ ! -S "/tmp/.X11-unix/X${display_num}" ]; then
    echo "Xvfb did not start; see /tmp/xvfb.log" >&2
    exit 1
fi

# 2. Window manager, so application windows are mapped and decorated.
if ! pgrep -x openbox > /dev/null; then
    echo "Starting Openbox"
    openbox > /tmp/openbox.log 2>&1 &
fi

# 3. VNC server on port 5900. It has no password; docker-compose.yml publishes
#    the port on the host loopback interface only.
if ! pgrep -x x11vnc > /dev/null; then
    echo "Starting x11vnc on port 5900"
    x11vnc -display "${DISPLAY}" -nopw -forever -shared -rfbport 5900 \
        > /tmp/x11vnc.log 2>&1 &
fi

# 4. noVNC web client on port 6080, proxying to the VNC server.
if ! pgrep -f websockify > /dev/null; then
    echo "Starting noVNC on port 6080"
    websockify --web /usr/share/novnc 6080 localhost:5900 > /tmp/novnc.log 2>&1 &
fi

echo "Virtual desktop ready: http://localhost:6080"
