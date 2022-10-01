# syntax = docker/dockerfile:experimental
FROM osrf/ros:melodic-desktop-full

############################## SYSTEM PARAMETERS ##############################
## Base arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash

## NVIDIA GraphicsCard parameter
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics, utility, compute

## Setup users and groups
RUN groupadd --gid ${GID} ${GROUP} \
  && useradd --gid ${GID} --uid ${UID} -ms ${SHELL} ${USER} \
  && mkdir -p /etc/sudoers.d \
  && echo "${USER}:x:${UID}:${UID}:${USER},,,:$HOME:${shell}" >> /etc/passwd \
  && echo "${USER}:x:${UID}:" >> /etc/group \
  && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
  && chmod 0440 "/etc/sudoers.d/${USER}"

## Replace apt urls
# Taiwan
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list
# TKU
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw/@g' /etc/apt/sources.list

############################### INSTALL & SETUP ###############################
## Install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  sudo htop git wget curl unzip \
  # Shell
  byobu zsh \
  terminator \
  dbus-x11 libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
  # Editing tools
  nano vim \
  gnome-terminal libcanberra-gtk-module libcanberra-gtk3-module \
  # Work tools
  cmake libgoogle-glog-dev libgflags-dev \
  libatlas-base-dev libeigen3-dev libsuitesparse-dev \
  libcap-dev ros-melodic-velodyne \
  python3-pip python3-dev python3-setuptools \
  python-pip python-catkin-tools python-setuptools \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists

WORKDIR /home/${USER}/.tmp

## setup custom configuration
COPY config .

## ROS Arguments
# Is the computer master or slave in ROS
ARG ROS_TYPE=MASTER
# ARG ROS_MASTER_IP=10.42.0.1
# ARG ROS_SLAVE_IP=10.42.0.2

ARG ROS_MASTER_IP=localhost
ARG ROS_SLAVE_IP=localhost

## Favorite shell when using byobu
ARG BYOBU_SHELL=zsh

## Set User configuration
RUN bash ./run_setup_script.sh \
  && rm -rf /home/${USER}/.tmp

## Copy entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod 777 /entrypoint.sh

WORKDIR /home/${USER}

## install LeGO-LOAM dep
RUN wget -O ./gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip \
  && unzip gtsam.zip && cd ./gtsam-4.0.0-alpha2 \
  && mkdir build && cd build \
  && cmake .. \
  && make install

## install A-LOAM dep
RUN wget -O ./ceres-solver.tar.gz  http://ceres-solver.org/ceres-solver-2.1.0.tar.gz \
  && tar zxf ceres-solver.tar.gz \
  && mkdir ceres-bin && cd ceres-bin \
  && cmake ../ceres-solver-2.1.0 \
  && make && make test && make install

RUN wget -O ./pcl.zip https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.1.zip \
  && unzip pcl.zip && cd pcl-pcl-1.12.1 && mkdir build && cd build \
  && cmake .. && make && make install

## Switch user to ${USER}
USER ${USER}

RUN sudo mkdir work

## Make SSH available
EXPOSE 22

## Switch to user's HOME folder
WORKDIR /home/${USER}/work

# ENTRYPOINT ["byobu"]
 ENTRYPOINT ["/entrypoint.sh", "byobu"]
# ENTRYPOINT ["/entrypoint.sh", "bash"]
# ENTRYPOINT ["/entrypoint.sh", "terminator"]
# ENTRYPOINT ["/entrypoint.sh", "zsh"]
