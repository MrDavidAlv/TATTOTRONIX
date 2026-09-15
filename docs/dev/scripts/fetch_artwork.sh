#!/usr/bin/env bash
# Fetch the official ROS logo into docs/dev/artwork/.
#
# The mark is published by ros-infrastructure/artwork under CC BY-NC 4.0 and is
# covered by the ROS trademark policy at https://www.ros.org/blog/media/. This
# repository is Apache-2.0, so the file is fetched rather than vendored: a
# non-commercial licence cannot ride along inside an Apache-2.0 tree. Nothing in
# the pipeline is specific to it - `rospath.from_image()` takes any mask.
set -euo pipefail

DEST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/artwork"
URL="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/ros_logo.svg"

mkdir -p "$DEST"
curl -sSL --fail -m 60 "$URL" -o "$DEST/ros_logo.svg"
echo "fetched $DEST/ros_logo.svg"
echo "  source   ros-infrastructure/artwork"
echo "  licence  CC BY-NC 4.0, plus the ROS trademark policy"
