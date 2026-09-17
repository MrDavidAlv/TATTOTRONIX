# syntax=docker/dockerfile:1

# TATTOTRONIX in a container.
#
# The environment this project needs is fragile enough to be worth pinning: the
# README's Known Issues carries a whole entry about an unrelated toolchain
# shadowing the ROS Python and breaking xacro in ways that look like something
# else. A container removes that class of problem, and it is the same image CI
# runs, so "works on my machine" and "passes the build" stop being different
# claims.
#
#   docker compose build
#   docker compose run --rm dev                 # shell, no GUI
#   docker compose run --rm dev colcon test     # or run anything directly
#   docker compose run --rm gui ros2 launch tattotronix_gazebo draw.launch.py
#
# Two stages. `base` installs dependencies and nothing else, so editing source
# does not reinstall ROS packages. `dev` adds the workspace.

ARG ROS_DISTRO=humble

# ---------------------------------------------------------------- base ------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# cairosvg reaches libcairo through ctypes, so the library has to be present
# even though nothing links against it at build time.
#
# ros2controlcli is what `ros2 control list_controllers` comes from. It is a
# debugging tool rather than a dependency, so no package.xml declares it and
# rosdep will not pull it in; without it the first thing anyone reaches for when
# a controller does not come up is missing.
#
# xvfb and ffmpeg are for docs/scripts/record_simulation.sh, which runs the
# simulation on a nested X server precisely so it does not need a real display.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        libcairo2 \
        xvfb \
        ffmpeg \
        ros-${ROS_DISTRO}-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Only the manifests, so a source edit does not invalidate the dependency
# layer. rosdep reads them and installs exactly what the packages declare,
# which keeps this file from drifting into a hand-maintained apt list.
COPY src/tattotronix_control/package.xml     src/tattotronix_control/package.xml
COPY src/tattotronix_description/package.xml src/tattotronix_description/package.xml
COPY src/tattotronix_gazebo/package.xml      src/tattotronix_gazebo/package.xml
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

COPY docs/scripts/requirements.txt docs/scripts/requirements.txt
RUN pip3 install --no-cache-dir -r docs/scripts/requirements.txt

# ----------------------------------------------------------------- dev ------
FROM base AS dev
ARG ROS_DISTRO
ARG UID=1000
ARG GID=1000

# A user whose ids match the host's, so files this container writes into the
# mounted workspace are not left owned by root.
RUN groupadd -g "${GID}" -o dev \
    && useradd -m -u "${UID}" -g "${GID}" -o -s /bin/bash dev \
    && mkdir -p /ws && chown -R "${UID}:${GID}" /ws

COPY --chown=${UID}:${GID} . /ws
RUN chown -R "${UID}:${GID}" /ws
USER dev

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --symlink-install --event-handlers console_direct+

COPY --chown=${UID}:${GID} docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
