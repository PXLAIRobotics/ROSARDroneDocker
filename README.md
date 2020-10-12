# ROSARDroneDocker
This repository contains multiple Dockerfiles and a few bash scripts to create a ROS Kinetic development environment for the Parrot AR.Drone 2.0 completely encapsulated in one container. Depending on the available graphics acceleration, it uses Nvidia or DRI.


## Building and running the container

The script to build all images is:
```bash
 $ ./0001_build_images.sh
```
It will check if there's an active Nvidia OpenGL driver or not and will build all the necessary images.


The following script will run the AR.Drone container:
```bash
 $ 
002_run_ardrone_container.sh 
```

## Gazebo worlds
There are several Gazebo worlds included:

The standard test world:
```bash
 $ roslaunch cvg_sim_gazebo ardrone_testworld.launch
```

Glasshouse
```bash
 $ $roslaunch pxl_drones_week glasshouse.launch
```

## Parrot control packages
There are two ROS package to control the UAV.

```bash
 $ rosrun motion_controller main.py
```

```bash
 $ rosrun manual_controller main.py
```
