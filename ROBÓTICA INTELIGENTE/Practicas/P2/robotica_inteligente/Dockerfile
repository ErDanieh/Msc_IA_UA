FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} docker
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} docker
RUN usermod -a -G dialout docker
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

## Dependencias
RUN apt-get update &&\
    apt-get install -y \
    ros-noetic-effort-controllers* \
    ros-noetic-ackermann-msgs ros-noetic-hector-gazebo \
    ros-noetic-moveit* \
    ros-noetic-soem \
    ros-noetic-socketcan-interface \
    ros-noetic-joint-trajectory-controller

USER docker
# Change HOME environment variable
ENV HOME /home/docker
RUN mkdir -p ${HOME}/catkin_ws/src

# Copiando repositorio al work_space de la docker
COPY ./src ${HOME}/catkin_ws/src


# Compilando el repositorio
RUN . /opt/ros/noetic/setup.sh && \
    cd ${HOME}/catkin_ws && \
    catkin_make

## permisos chmod +x
USER root
RUN cd ${HOME}/catkin_ws/src/robotica_inteligente/scripts/ && sudo chmod +x spawn_random_position.py 
RUN cd ${HOME}/catkin_ws/src/ackermann_vehicle/ackermann_vehicle_gazebo/scripts/ && sudo chmod +x ackermann_controller

# set up environment
COPY ./update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

USER docker
# Configura la variable de entorno DISPLAYWORKDIR
ENV DISPLAY=:0    
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility
WORKDIR ${HOME}/catkin_ws
