#!/usr/bin/env bash
# Take a still of RViz on a nested X server.
#
# The desktop cannot be captured programmatically on this machine: GNOME
# refuses ScreenshotArea and Wayland hands x11grab a black frame. So RViz is
# run on an Xvfb display of its own, which is a real X server that x11grab
# reads directly, and which no other session can land on top of.
#
# It brings up only what a picture needs - the description, a joint state and
# move_group - and never a simulator, so nothing here can collide with a
# simulation the developer already has running.
#
#   docs/scripts/capture_rviz.sh <config.rviz> <output.png> [hardware]
#
# Run it inside the container: docker compose run --rm ci bash -lc '...'
set -euo pipefail

CONFIG="${1:?usage: capture_rviz.sh <config.rviz> <output.png> [hardware]}"
OUT="${2:?usage: capture_rviz.sh <config.rviz> <output.png> [hardware]}"
HARDWARE="${3:-mock}"
# A picture of the collision geometry needs the description and nothing else.
# Set NO_MOVE_GROUP=1 for it: starting a planner it will not use only adds a
# panel over the thing being photographed.
NO_MOVE_GROUP="${NO_MOVE_GROUP:-0}"
DISP=":99"
W=1600
H=1000
SETTLE="${SETTLE:-18}"

cleanup() {
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT
PIDS=()

# ROS setup scripts read variables they have not set, so -u has to stand down
# for exactly as long as it takes to source them.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

Xvfb "$DISP" -screen 0 "${W}x${H}x24" +extension GLX +extension RANDR -nolisten tcp \
    >/tmp/xvfb.log 2>&1 &
PIDS+=($!)
for _ in $(seq 30); do DISPLAY="$DISP" xdpyinfo >/dev/null 2>&1 && break; sleep 1; done
DISPLAY="$DISP" xdpyinfo >/dev/null 2>&1 || { echo "X server did not start" >&2; exit 1; }
export DISPLAY="$DISP"

xacro src/tattotronix_description/urdf/tattotronix.urdf.xacro "hardware:=${HARDWARE}" \
    > /tmp/capture.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/capture.urdf >/tmp/rsp.log 2>&1 &
PIDS+=($!)
ros2 run joint_state_publisher joint_state_publisher >/tmp/jsp.log 2>&1 &
PIDS+=($!)
if [ "$NO_MOVE_GROUP" != "1" ]; then
    ros2 launch tattotronix_moveit_config move_group.launch.py \
        "hardware:=${HARDWARE}" use_sim_time:=false >/tmp/mg.log 2>&1 &
    PIDS+=($!)
    for _ in $(seq 40); do
        ros2 service list 2>/dev/null | grep -q plan_kinematic_path && break
        sleep 1
    done
fi

rviz2 -d "$CONFIG" >/tmp/rviz.log 2>&1 &
PIDS+=($!)

# RViz draws an empty window first and fills it as the planning scene arrives.
# Grabbing too early gives a grey rectangle that looks like a broken config.
sleep "$SETTLE"

mkdir -p "$(dirname "$OUT")"
ffmpeg -loglevel error -y -f x11grab -video_size "${W}x${H}" -i "$DISP" \
    -frames:v 1 "$OUT"
echo "wrote $OUT"
