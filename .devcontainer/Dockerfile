FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential cmake curl git iproute2 iputils-ping \
    libgl1 libglib2.0-0 python3-colcon-common-extensions python3-pip \
    python3-rosdep python3-venv ros-jazzy-cv-bridge ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py ros-jazzy-image-transport \
    ros-jazzy-rmw-fastrtps-cpp sudo \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu images can already have a UID-1000 user. Preserve its identity rather
# than deleting it, and reuse it for the development account when present.
RUN if id -u ubuntu >/dev/null 2>&1; then \
      usermod --login ros --home /home/ros --move-home ubuntu; \
      groupmod --new-name ros ubuntu; \
    else \
      useradd --create-home --shell /bin/bash ros; \
    fi \
    && usermod --shell /bin/bash ros \
    && echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros \
    && python3 -m venv --system-site-packages /opt/venv \
    && mkdir -p /home/ros/.ros /home/ros/.cache/pip \
       /workspaces/fall-detection-robot/ros_ws/{src,build,install,log} \
    && chown -R ros:ros /opt/venv /home/ros \
       /workspaces/fall-detection-robot \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

COPY .devcontainer/scripts/entrypoint.sh /usr/local/bin/dev-entrypoint
COPY .devcontainer/scripts/make_dds_profile.py /usr/local/bin/make-dds-profile
COPY .devcontainer/scripts/ros-shell.sh /etc/profile.d/ros-dev.sh
RUN chmod 0755 /usr/local/bin/dev-entrypoint /usr/local/bin/make-dds-profile \
    && echo 'source /etc/profile.d/ros-dev.sh' >> /home/ros/.bashrc

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:${PATH}"
ENV ROS_DOMAIN_ID=0
ENV ROS_DDS_PEERS=127.0.0.1
ENV ROS_DDS_ADDRESS=127.0.0.1
USER ros
WORKDIR /workspaces/fall-detection-robot
ENTRYPOINT ["/usr/local/bin/dev-entrypoint"]
CMD ["sleep", "infinity"]
