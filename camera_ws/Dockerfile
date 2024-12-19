# Use TensorFlow 2.17.0 RC0 as the base image
FROM tensorflow/tensorflow:2.17.0rc0

RUN apt update && apt install locales \
    locale-gen en_US en_US.UTF-8 \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \ 
    export LANG=en_US.UTF-8
    
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8
    
RUN apt install software-properties-common \
    add-apt-repository universe
    
RUN apt update && sudo apt install curl -y \
    url -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
RUN apt update && apt upgrade

RUN apt install ros-foxy-desktop python3-argcomplete

RUN apt install ros-dev-tools

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

ENV ROS_DOMAIN_ID=9

# Create a workspace for camera_ws
RUN mkdir -p /home/ros/camera_ws/src
WORKDIR /home/ros/camera_ws
