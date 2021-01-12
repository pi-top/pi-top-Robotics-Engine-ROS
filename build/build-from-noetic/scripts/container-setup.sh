#!/bin/bash

# exit when any command fails
set -e
set -o xtrace

chown -R root /tmp
# fix GPG key issue with ROS Noetic
apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
  --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get clean && apt-get update

# update apt registry
apt-get --fix-broken install  # get this recommendation if trying to install packages
apt-get update
apt-get upgrade -y

# install build tools like gcc (ROS needs it)
apt-get install -y build-essential

# install python3 tools
apt-get install -y python3-pip python3-opencv python3-yaml
apt-get install -y libv4l-dev

# various tools for development purposes
apt-get install -y nano git wget i2c-tools

# get pi-top SDK libs
cd /home/
git clone https://github.com/pi-top/pi-top-Python-Common-Library
git clone https://github.com/pi-top/pi-top-Python-SDK

# install common library
cd pi-top-Python-Common-Library
pip3 install .
python3 setup.py install

# install SDK
cd /home/pi-top-Python-SDK/
pip3 install .
python3 setup.py install

# install ros packages
apt-get install -y ros-noetic-tf ros-noetic-tf2-tools
apt-get install -y ros-noetic-usb-cam ros-noetic-image-view ros-noetic-image-transport
apt-get install -y ros-noetic-compressed-image-transport
apt-get install -y ros-noetic-cv-bridge # this is a biggy
#apt-get install -y ros-noetic-cv-camera  # build this one from our own source
apt-get install -y ros-noetic-opencv-apps  # face tracking, people tracking and more
apt-get install -y python3-rosdep

# install hostname resolver
apt-get install -y avahi-daemon

# install tools for server
apt-get install -y python3-tornado ros-noetic-rosbridge-suite nginx
# Configure nginx server root to ros-web-ui directory
sed -i 's/root \/var\/www\/html/root \/home\/catkin_ws\/src\/robotics_web_controller/' /etc/nginx/sites-enabled/default
service nginx restart

# install python packages
pip3 install cython gpiozero smbus2 numpy
pip3 install imageio
pip3 install PyV4L2Camera
pip3 install simple_pid

#initialise catkin workspace
source /ros_entrypoint.sh
cd /home/catkin_ws/src || exit
catkin_init_workspace

# chmod python files so they can execute
chmod +x /home/catkin_ws/src/robotics_apps/src/controller.py
chmod +x /home/catkin_ws/src/robotics_apps/src/line_follower_node.py
chmod +x /home/catkin_ws/src/robotics_apps/src/math_functions.py
chmod +x /home/catkin_ws/src/robotics_apps/src/vision_functions.py
chmod +x /home/catkin_ws/src/robotics_apps/src/vision_functions.py
chmod +x /home/catkin_ws/src/robotics_base/chassis_imu/src/imu_publisher.py
chmod +x /home/catkin_ws/src/robotics_base/chassis_move/src/chassis_move.py
chmod +x /home/catkin_ws/src/robotics_base/chassis_odom/src/odom_publisher.py
chmod +x /home/catkin_ws/src/robotics_base/chassis_servos/src/chassis_servos.py

# need ros-noetic-web-video-server and async_web_server_cpp but cant find it on noetic
cd /home/catkin_ws/src || exit
git clone https://github.com/GT-RAIL/async_web_server_cpp
git clone https://github.com/RobotWebTools/web_video_server
cd /home/catkin_ws/ || exit
catkin_make
source /home/catkin_ws/devel/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc

# install ssh server for code deployment
apt-get install -y openssh-server
mkdir /var/run/sshd
# change root password to pi-top
echo 'root:pi-top' | chpasswd
sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
echo "export VISIBLE=now" >> /etc/profile
service ssh restart

# comment out for development purposes
#rm -rf /var/lib/apt/lists/
