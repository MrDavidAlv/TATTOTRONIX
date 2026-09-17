#!/usr/bin/env bash
#
# Record the arm drawing the logo in Gazebo and RViz.
#
# Runs the simulation on a virtual X server rather than the desktop. Under
# Wayland the compositor never puts window contents in the X root window, so a
# screen grab of the real session returns a black frame, and GNOME refuses the
# programmatic screenshot interface outright. A nested X server sidesteps both:
# it is a real X server, x11grab reads it directly, and on this machine it still
# gets hardware GL - RViz reports 4.6 on :99 - so nothing renders in software.
#
#   docs/scripts/record_simulation.sh                  # both views, whole drawing
#   docs/scripts/record_simulation.sh --view rviz      # only RViz
#   docs/scripts/record_simulation.sh --seconds 30     # a short check
#
# Writes docs/figures/simulation-rviz.{mp4,gif} and simulation-gazebo.{mp4,gif}.
# Separate files rather than one split screen: half a 1920 frame each is half the
# detail, and the two views answer different questions.
# No `set -u`: the ROS setup scripts read variables they have not set.
set -eo pipefail

SECONDS_TO_RECORD=""
SPEED=1.0
DISPLAY_NUM=99
VIEW=both
W=1920
H=1080
FPS=30
CRF=18

while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) SECONDS_TO_RECORD="$2"; shift 2 ;;
    --speed)   SPEED="$2"; shift 2 ;;
    --display) DISPLAY_NUM="$2"; shift 2 ;;
    --view)    VIEW="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Each view gets its own full-screen run. Sharing one run would mean either a
# split screen, which halves the detail in each half, or raising one window over
# the other partway through, which loses whichever view is underneath.
if [ "$VIEW" = both ]; then
  "$0" --view rviz   --speed "$SPEED" ${SECONDS_TO_RECORD:+--seconds "$SECONDS_TO_RECORD"}
  "$0" --view gazebo --speed "$SPEED" ${SECONDS_TO_RECORD:+--seconds "$SECONDS_TO_RECORD"}
  exit 0
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FIG="$REPO/docs/figures"
WORK="$(mktemp -d)"
DISP=":$DISPLAY_NUM"

sim_processes() {
  # Matching on the command line finds the grep itself, this script, and the
  # shell that invoked it, because the pattern sits in all of their arguments.
  # The bracketed first letters stop the pattern matching its own grep, and the
  # name filter drops the script and its wrapper.
  ps -eo pid=,args= 2>/dev/null |
    grep -E '[i]gn gazebo|[r]viz2 -d|[d]raw_logo|[p]arameter_bridge|[r]obot_state_publisher' |
    grep -v 'record_simulation.sh' |
    awk -v me="$$" '$1 != me {print $1}'
}

cleanup() {
  set +e
  for pid in $FFMPEG_PIDS; do kill -INT "$pid" 2>/dev/null; wait "$pid" 2>/dev/null; done
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 3
  # parameter_bridge outlives the launch it belonged to, and a stale one keeps
  # publishing /clock. Two clocks make the controller see time jump backwards
  # and the next run never gets its trajectory accepted, so it is killed by name
  # rather than left to the launch teardown.
  for pid in $(sim_processes); do kill -9 "$pid" 2>/dev/null; done
  [ -n "$XVFB_PID" ] && kill "$XVFB_PID" 2>/dev/null
  rm -rf "$WORK"
}
trap cleanup EXIT

# Refuse to stack a simulation on top of one already running: two of them fight
# over /clock and over the controller manager, and the failure looks like a
# timeout rather than a conflict.
if [ -n "$(sim_processes)" ]; then
  echo "a simulation is already running:" >&2
  ps -o pid=,args= -p $(sim_processes | tr '\n' ' ') 2>/dev/null | cut -c1-110 >&2
  echo "stop it first, or kill those pids." >&2
  exit 1
fi

# OSS CAD Suite ships a python3 without PyYAML, which breaks xacro.
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v oss-cad-suite | paste -sd:)"
export PATH

echo "starting X server on $DISP ..."
Xvfb "$DISP" -screen 0 "${W}x${H}x24" +extension GLX +extension RANDR -nolisten tcp \
  > "$WORK/xvfb.log" 2>&1 &
XVFB_PID=$!
for _ in $(seq 60); do DISPLAY="$DISP" xdpyinfo >/dev/null 2>&1 && break; sleep 1; done
DISPLAY="$DISP" xdpyinfo >/dev/null 2>&1 || { echo "X server did not start" >&2; exit 1; }
echo "  $(DISPLAY=$DISP glxinfo -B 2>/dev/null | grep -i 'opengl renderer' || echo 'no GL info')"

export DISPLAY="$DISP"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$REPO"
# shellcheck disable=SC1091
source install/setup.bash

echo "launching the simulation ..."
ros2 launch tattotronix_gazebo draw.launch.py "speed:=$SPEED" \
  "rviz_config:=$REPO/src/tattotronix_gazebo/rviz/recording.rviz" \
  > "$WORK/sim.log" 2>&1 &
LAUNCH_PID=$!

echo "waiting for the controller to accept the trajectory ..."
for _ in $(seq 180); do
  grep -qi "drawing" "$WORK/sim.log" && break
  grep -qi "rejected the trajectory\|no action server" "$WORK/sim.log" && {
    tail -20 "$WORK/sim.log" >&2; echo "the controller refused the goal" >&2; exit 1; }
  sleep 1
done
grep -qi "drawing" "$WORK/sim.log" || { tail -30 "$WORK/sim.log" >&2; exit 1; }

# There is no window manager on the nested server, so windows are placed
# directly.
#
# Qt sizes its window after mapping it, so a single move-and-resize lands before
# the application has finished deciding how big it wants to be. The placement is
# applied, checked, and repeated.
place() {                      # id
  local id="$1"
  for _ in 1 2 3 4; do
    xdotool windowmove --sync "$id" 0 0 2>/dev/null || true
    xdotool windowsize --sync "$id" "$W" "$H" 2>/dev/null || true
    xdotool windowraise "$id" 2>/dev/null || true
    sleep 1
    local g
    g=$(xdotool getwindowgeometry --shell "$id" 2>/dev/null |
        awk -F= '/^WIDTH/{w=$2} /^HEIGHT/{h=$2} END{print w"x"h}')
    [ "$g" = "${W}x${H}" ] && return 0
  done
  echo "  warning: window settled at $g, wanted ${W}x${H}" >&2
}

find_window() {                # name-regex
  local id
  for _ in $(seq 40); do
    id=$(xdotool search --name "$1" 2>/dev/null | tail -1)
    [ -n "$id" ] && { echo "$id"; return 0; }
    sleep 1
  done
  return 1
}

total=$(grep -oP 'sending \d+ points, \K\d+' "$WORK/sim.log" | tail -1)
[ -n "$total" ] || total=560
duration="${SECONDS_TO_RECORD:-$(( total + 15 ))}"

# RViz keeps its property docks whatever `Hide Left Dock` says once the saved
# window state is gone, so the docks are cropped out of the recording instead of
# fought with. The numbers are the 3D viewport inside a 1920x1080 RViz window.
case "$VIEW" in
  rviz)   name="RViz"; crop="1428:1016:492:58" ;;
  gazebo) name="^Gazebo$"; crop="" ;;
  *) echo "--view must be rviz, gazebo or both" >&2; exit 2 ;;
esac

id=$(find_window "$name") || { echo "no $VIEW window appeared" >&2; exit 1; }
place "$id"

if [ "$VIEW" = gazebo ]; then
  # Move the camera in rather than editing the world: the world's own view is
  # the one people get when they open it interactively, and a recording should
  # not quietly redefine that.
  ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera \
    --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'pose: {position: {x: 0.02, y: -0.42, z: 1.12},
                  orientation: {x: 0.30, y: 0.13, z: 0.35, w: 0.87}}' \
    >/dev/null 2>&1 || echo "  (could not move the Gazebo camera; using the world view)"
fi
sleep 4

echo "recording ${duration}s of ${W}x${H} at ${FPS} fps from $VIEW ..."
ffmpeg -loglevel error -y -f x11grab -framerate "$FPS" -video_size "${W}x${H}" \
  -i "$DISP" -t "$duration" -c:v libx264 -preset medium -crf "$CRF" \
  -pix_fmt yuv420p "$WORK/raw.mp4" &
FFMPEG_PIDS="$!"
wait $FFMPEG_PIDS
FFMPEG_PIDS=""

mkdir -p "$FIG"
base="$FIG/simulation-$VIEW"
# Real time is nine minutes of drawing at 6 mm/s. The published clip is sped up
# so it can sit in a README; the factor is printed and captioned, never hidden.
target=26
factor=$(python3 -c "print(max(1.0, $duration / $target))")
echo "speeding up x$(printf '%.0f' "$factor") ..."
vf="setpts=PTS/$factor"
[ -n "$crop" ] && vf="$vf,crop=$crop"
vf="$vf,scale=1440:-2:flags=lanczos"
ffmpeg -loglevel error -y -i "$WORK/raw.mp4" -vf "$vf" -an \
  -c:v libx264 -crf "$CRF" -preset slow -pix_fmt yuv420p "$base.mp4"
ffmpeg -loglevel error -y -i "$base.mp4" \
  -vf "fps=12,scale=900:-1:flags=lanczos,palettegen=stats_mode=diff" "$WORK/pal.png"
ffmpeg -loglevel error -y -i "$base.mp4" -i "$WORK/pal.png" \
  -lavfi "fps=12,scale=900:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=3" \
  "$base.gif"

echo "wrote $base.mp4 ($(du -h "$base.mp4" | cut -f1))"
echo "wrote $base.gif ($(du -h "$base.gif" | cut -f1))"
