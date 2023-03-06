FROM ros:melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full


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

   
# RUN sudo apt install -y ros-melodic-mav-msgs ros-melodic-rosmon python-wstool python-jinja2  ros-melodic-geographic-msgs  libgeographic-dev  geographiclib-tools

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

RUN sudo apt update -y


WORKDIR /home/$USERNAME/
# RUN git clone -b v0.5 https://github.com/stevenlovegrove/Pangolin.git && git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git

# WORKDIR /home/$USERNAME/Pangolin
# RUN mkdir build 
# WORKDIR /home/$USERNAME/Pangolin/build
# RUN cmake ..   
# RUN cmake --build .

RUN sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

WORKDIR /home/$USERNAME/

RUN tar zxf ceres-solver-2.1.0.tar.gz
RUN mkdir ceres-bin && cd ceres-bin
RUN cmake ../ceres-solver-2.1.0
RUN make -j8
RUN sudo make install

# WORKDIR /home/$USERNAME/opencv/
# RUN mkdir build
# WORKDIR /home/$USERNAME/opencv/build
# RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/home/$USERNAME/opencv_contrib/modules/ ..
# RUN make -j7 && sudo make install 

WORKDIR /home/$USERNAME/
RUN rosdep update




