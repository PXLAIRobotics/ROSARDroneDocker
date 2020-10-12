#!/bin/bash

if ! glxinfo -v COMMAND &> /dev/null
then
    echo "glxinfo not found!"
    echo
    echo "Please install it via:"
    echo "   sudo apt install mesa-utils"
    exit -1
fi

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`

# Building the ubuntu 16.04 base image
if [ $vendor == "NVIDIA" ]; then
    (cd ./01a_nvidia_opengl_cuda; ./01_build_image.sh)
else
    (cd ./01b_opengl_direct_rendering; ./01_build_image.sh)
fi

# Building the ROS AR.Drone image
(cd ./02_ar_drone; ./01_build_image.sh)
