#!/usr/bin/env bash
# Bring move_group up against the description on mock hardware, with a
# robot_state_publisher and a joint state, and run moveit_check.py against it.
# Run inside the container:  docker compose run --rm dev docs/scripts/moveit_check.sh
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
PIDS=()
trap 'for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done' EXIT
xacro src/tattotronix_description/urdf/tattotronix.urdf.xacro hardware:=mock > /tmp/check.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/check.urdf >/tmp/rsp.log 2>&1 &
PIDS+=($!)
ros2 run joint_state_publisher joint_state_publisher >/tmp/jsp.log 2>&1 &
PIDS+=($!)
ros2 launch tattotronix_moveit_config move_group.launch.py hardware:=mock \
    use_sim_time:=false >/tmp/mg.log 2>&1 &
PIDS+=($!)
python3 docs/scripts/moveit_check.py
