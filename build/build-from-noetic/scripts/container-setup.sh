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

# install build tools like gcc (ROS needs it)
apt-get install -y build-essential

# install python3 tools
apt-get install -y python3-pip python3-opencv python3-yaml
apt-get install -y libv4l-dev

# random tools for development purposes
apt-get install -y nano git
#apt-get install -y wget i2c-tools

# install ros packages
apt-get install -y ros-noetic-tf
apt-get install -y ros-noetic-tf2-tools
apt-get install -y ros-noetic-usb-cam
apt-get install -y ros-noetic-image-view
apt-get install -y ros-noetic-image-transport
apt-get install -y ros-noetic-compressed-image-transport
apt-get install -y ros-noetic-cv-bridge # this is a biggy
#apt-get install -y ros-noetic-cv-camera  # build this one from our own source
apt-get install -y ros-noetic-opencv-apps  # face tracking, people tracking and more
apt-get install -y python3-rosdep
apt-get install -y ros-noetic-pid

# install hostname resolver (this doesn't work to fix for now but might with some config)
apt-get install -y avahi-daemon

# install tools for server
apt-get install -y python3-tornado ros-noetic-rosbridge-suite nginx
# Configure nginx server root to ros-web-ui directory
sed -i 's/root \/var\/www\/html/root \/home\/catkin_ws\/src\/robotics_web_controller/' /etc/nginx/sites-enabled/default
service nginx restart

# install python packages
pip3 install cython
pip3 install gpiozero smbus2 numpy
pip3 install imageio
pip3 install PyV4L2Camera

#initialise catkin workspace
source /ros_entrypoint.sh
cd /home/catkin_ws/src || exit
catkin_init_workspace
# chmod python files so they can execute
#chmod +x /home/catkin_ws/src/ros-rover/chmod_python_files.sh
#bash /home/catkin_ws/src/ros-rover/chmod_python_files.sh

# need ros-noetic-web-video-server  and async_web_server_cpp but cant find it on noetic
# could put this in its own directory or into main ROS directory in future
cd /home/catkin_ws/src || exit
git clone https://github.com/GT-RAIL/async_web_server_cpp
git clone https://github.com/RobotWebTools/web_video_server
cd /home/catkin_ws/ || exit
catkin_make
source /home/catkin_ws/devel/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc

# install ssh server and configure so pycharm can deploy
apt-get install -y openssh-server
mkdir /var/run/sshd
echo 'root:pi-top' | chpasswd
sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
echo "export VISIBLE=now" >> /etc/profile
service ssh restart

# comment out for development purposes
#rm -rf /var/lib/apt/lists/
