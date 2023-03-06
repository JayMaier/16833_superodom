## VINS testing repo

Containerized repo that runs VINS Fusion in ROS Noetic.  

-----------------------
## Building the docker-image
The requirements for this are `docker` and `docker-compose`. To install this repository, please follow the commands below.

```console
git clone -b https://github.com/JayMaier/16833_superodom.git
cd 16833_superodom
git submodule update --init --recursive
cd docker
docker-compose build --
```

Takes about a half hour on Jay's laptop


```console
export HOSTNAME=$HOSTNAME
docker-compose up
```

Open another terminal and attach to the container (if it whines, try giving it sudo):

```console
docker exec -it superodom bash
```


---------------------
## Building the Packages
We need to build the code once for the first time. Once inside the docker container do,
```console
cd superodom_ws/ros_ws
catkin_make
source ~/.bashrc
```


--------------

## Running VINS Fusion

Can test run stock VINS on a EUROC log to verify instalation:
```console
roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
(optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

--------------------------

## Extra Information

- The docker images don't run on NVIDIA GPUs, ensure you're primary display manager is not NVIDIA. Set the GPU profile to either `on-demand` or `Intel` on nvidia-smi on ubuntu. The package will throw a X11 error otherwise.
- The ros-master is shared with the PC outside the docker. You can run any other package outside the container and send it to the ROS running inside the container.
