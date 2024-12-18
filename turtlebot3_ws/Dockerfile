FROM ghcr.io/sloretz/ros:humble-desktop

# Update package lists and install additional packages
RUN apt-get update && apt-get install -y \
    vim \
    git \
    ros-humble-navigation2 \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*
    
RUN sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential

# Update package list and install TurtleBot3 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Run custom scripts or commands during build
RUN echo "Custom setup complete."
