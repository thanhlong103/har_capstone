FROM dustynv/ros:foxy-desktop-l4t-r35.4.1
WORKDIR /home


# Install ROS 2 packages
#RUN apt-get update && apt-get install -y ros-foxy-ros-base python3-argcomplete

#RUN apt-get update && apt-get install -y python3-rosdep2 python3-tk

#RUN apt-get install -y ros-foxy-turtlebot3-description ros-foxy-gazebo-ros-pkgs

#RUN apt-get update && apt-get install -y ros-dev-tools

# --- OPEN CV 3.4.12 INSTALLATION ---

# Install dependencies for building OpenCV
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libjpeg-dev \
    libtiff-dev \ 
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \ 
    libatlas-base-dev \
    gfortran \
    python3-dev \
    python3-numpy

# Clone OpenCV and OpenCV contrib
RUN git clone -b 3.4.12 https://github.com/opencv/opencv.git && \
    git clone -b 3.4.12 https://github.com/opencv/opencv_contrib.git

# Build OpenCV
WORKDIR /home/opencv
RUN mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/home/opencv_contrib/modules \
          -D BUILD_opencv_python3=ON \
          -D PYTHON_EXECUTABLE=$(which python3) \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# --- INITIALIZE APPLICATION ---
WORKDIR /home
RUN apt-get install -y nano tmux

#RUN apt-get update && apt-get install -y \
#    ros-foxy-rviz2 \
#    libogre-1.12-dev \
#    python3-pykdl   

#RUN apt-get install -y libopenblas-base libopenmpi-dev libomp-dev #??
#RUN apt-get install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools #??
#RUN apt-get install -y pyqt5-dev #??
#RUN apt-get install -y qt5-default  #??
#RUN apt-get -y install python3-pyqt5  #??

RUN apt-get install -y python3-pip 

RUN pip install \
    scipy \
    pykalman 
RUN apt-get install -y python-numpy

# Set up ~/.bashrc file

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=9" >> ~/.bashrc
RUN source ~/.bashrc

WORKDIR /home/ros2_ws
CMD ["bash"]
