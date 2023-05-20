FROM ros:humble

ARG USERNAME=appuser
ARG GROUP=appgroup
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash

# 必要なパッケージをインストール
RUN sudo apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Setup users and groups
RUN groupadd --gid ${GID} ${GROUP} \
    && useradd --gid ${GID} --uid ${UID} -ms ${SHELL} ${USERNAME} \
    && mkdir -p /etc/sudoers.d \
    && echo "${USERNAME}:x:${UID}:${UID}:${USERNAME},,,:$HOME:${shell}" >> /etc/passwd \
    && echo "${USERNAME}:x:${UID}:" >> /etc/group \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Switch user to ${USERNAME}
USER ${USERNAME}

# Switch to user's HOME folder
WORKDIR /home/${USERNAME}

# Expose ports for ROS2
EXPOSE 11311

# copy entrypoint
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
