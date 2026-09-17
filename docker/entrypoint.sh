#!/usr/bin/env bash
# Source ROS and the workspace, then run whatever was asked for.
#
# `set -e` but not `set -u`: the ROS setup scripts read variables they have not
# set, and failing on that would make the container unusable.
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

exec "$@"
