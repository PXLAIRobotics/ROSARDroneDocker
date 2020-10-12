FROM pxl_ra_base_ubuntu16.04:latest

USER root

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

# Add updated Gazebo ppa
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - |  apt-key add -

RUN apt-get update \
        && apt-get upgrade -y \ 
        && apt-get install -y ros-kinetic-desktop-full \
        && apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential \
        binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 \
        libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot \
        libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 \
        libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 \
        libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic \
        linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev

RUN apt-get install -y ros-kinetic-hector-* \
        && apt-get install -y ros-kinetic-ardrone-autonomy \
        && apt-get install -y python-pyside \
        && rosdep init

RUN rm -rf /var/lib/apt/lists/* 

USER user
WORKDIR /home/user
RUN rosdep update \
        && echo "source /opt/ros/kinetic/setup.bash" >> /home/user/.bashrc

RUN mkdir -p /home/user/Projects/ar_drone_ws/src 
WORKDIR /home/user/Projects/ar_drone_ws/src
COPY --chown=user:user ./pxl_drones_week.tar.gz /home/user/Projects/ar_drone_ws/src/pxl_drones_week.tar.gz 
RUN tar xvf ./pxl_drones_week.tar.gz
RUN git clone https://github.com/PXLAIRobotics/tum_simulator.git
WORKDIR /home/user/Projects/ar_drone_ws/
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /home/user/Projects/ar_drone_ws; catkin_make'
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /home/user/Projects/ar_drone_ws; catkin_make'
RUN echo "source /home/user/Projects/ar_drone_ws/devel/setup.bash --extend" >> /home/user/.bashrc

WORKDIR /home/user

RUN mkdir -p Projects/catkin_ws/src 
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'

RUN echo "source /home/user/Projects/catkin_ws/devel/setup.bash --extend" >> /home/user/.bashrc
