FROM ros:noetic
RUN apt-get update
ENV DEBIAN_FRONTEND=noninteractive 
RUN apt-get install -y \
    ros-noetic-desktop-full


# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id=$user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

   
# RUN sudo apt install -y ros-noetic-mav-msgs ros-noetic-rosmon python-wstool python-jinja2  ros-noetic-geographic-msgs  libgeographic-dev  geographiclib-tools

# RUN sudo apt-get install -y openssh-server python-dev python3-pip software-properties-common less vim gdb python-catkin-tools python-pip python-wheel nano

# RUN pip install numpy toml future

# RUN sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev

# RUN sudo apt install -y --no-install-suggests --no-install-recommends libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev

# RUN sudo apt install -y --no-install-suggests --no-install-recommends libc++-dev libglew-dev libeigen3-dev cmake libjpeg-dev libpng-dev 

# RUN sudo apt install -y --no-install-suggests --no-install-recommends libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev 

# ceres deps: cmake >3.10 eigen > 3.3 glog > 0.3.5 gflags suitesparse > 4.5.6 





RUN sudo apt install git-all -y
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all 

ENV entrypoint_container_path /docker-entrypoint/
ADD entrypoints/ $entrypoint_container_path/

# execute entrypoint script
RUN sudo chmod +x -R $entrypoint_container_path/




WORKDIR /home/$USERNAME/
# RUN git clone -b v0.5 https://github.com/stevenlovegrove/Pangolin.git && git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git

# WORKDIR /home/$USERNAME/Pangolin
# RUN mkdir build 
# WORKDIR /home/$USERNAME/Pangolin/build
# RUN cmake ..   
# RUN cmake --build .

RUN sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

WORKDIR /home/$USERNAME/superodom_ws


RUN sudo git clone -b 2.1.0 https://ceres-solver.googlesource.com/ceres-solver

WORKDIR /home/$USERNAME/superodom_ws/ceres-bin

# RUN sudo mkdir ceres-bin && cd ceres-bin
RUN sudo cmake ../ceres-solver
RUN sudo make -j8
RUN sudo make install

# installing vins fusion stuff


WORKDIR /home/$USERNAME/superodom_ws/ros_ws/src
RUN ls
RUN sudo git clone https://github.com/stevenf7/VINS-Fusion.git

# WORKDIR /home/$USERNAME/superodom_ws/ros_ws

# RUN rm -rf devel && rm -rf build
# RUN . /opt/ros/noetic/setup.sh && catkin_make 


WORKDIR /home/$USERNAME/superodom_ws
RUN sudo git clone -b develop https://github.com/borglab/gtsam.git
WORKDIR /home/$USERNAME/superodom_ws/gtsam/
RUN sudo mkdir build
WORKDIR /home/$USERNAME/superodom_ws/gtsam/build
RUN pwd
RUN ls -a ..
RUN sudo cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
RUN sudo make install 


RUN sudo apt update -y
WORKDIR /home/$USERNAME/
RUN rosdep update




