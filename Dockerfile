FROM ros:noetic-ros-core-focal

# create catkin workspace
ENV CATKIN_WS /home/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# add robotics engine source files
ADD src $CATKIN_WS/src/

# add .bashrc entries
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
RUN echo 'cd /home/catkin_ws' >> ~/.bashrc

# change shell so that we can use anything that has been "source"d
SHELL ["/bin/bash", "-c", "-l"]

# update apt registry
# RUN apt-get --fix-broken install
RUN apt-get update
RUN apt-get upgrade -y

# install build tools like gcc (ROS needs it)
RUN apt-get install -y build-essential

# install python3 tools
RUN apt-get install -y python3-pip \
                   python3-opencv \
                   python3-yaml \
                   libv4l-dev

# required for pillow install
RUN apt-get install -y libjpeg-dev zlib1g-dev

# various tools for development purposes
RUN apt-get install -y nano git wget i2c-tools

# get pi-top SDK libs
RUN cd /home/ \
    && git clone https://github.com/pi-top/pi-top-Python-Common-Library \
    && git clone https://github.com/pi-top/pi-top-Python-SDK

# install common library
RUN cd /home/pi-top-Python-Common-Library \
    && pip3 install . \
    && python3 setup.py install

# install SDK
# Need to remove scipy and matplotlib as dependancies as cannot install for unknown reasons
# These are only needed for IMU calibration using pi-top CLI so not required in the Docker container
RUN cd /home/pi-top-Python-SDK/ \
    && sed -i '/"matplotlib",/d' setup.py \
    && sed -i '/"scipy",/d' setup.py \
    && pip3 install . \
    && python3 setup.py install

# install ros packages
RUN apt-get install -y ros-noetic-tf \
                   ros-noetic-tf2-tools \
                   ros-noetic-usb-cam \
                   ros-noetic-image-view \
                   ros-noetic-image-transport \
                   ros-noetic-compressed-image-transport \
                   ros-noetic-cv-bridge \
                   ros-noetic-opencv-apps \
                   python3-rosdep \
                   ros-noetic-rosbridge-suite

# install tools for server
RUN apt-get install -y python3-tornado nginx

# Configure nginx server root to ros-web-ui directory
RUN sed -i 's/root \/var\/www\/html/root \/home\/catkin_ws\/src\/robotics_web_controller/' /etc/nginx/sites-enabled/default
RUN service nginx restart
RUN echo 'service nginx restart' >> ~/.bashrc

# install python packages
RUN pip3 install cython \
                 gpiozero \
                 smbus2 \
                 numpy \
                 imageio \
                 PyV4L2Camera \
                 simple_pid

# chmod python files so they can execute
RUN find /home/catkin_ws/src -type f -name "*.py" -exec chmod +x {} \;
#RUN chmod +x /home/catkin_ws/src/robotics_apps/line_follower/src/controller.py \
#    && chmod +x /home/catkin_ws/src/robotics_apps/line_follower/src/line_follower_node.py \
#    && chmod +x /home/catkin_ws/src/robotics_apps/line_follower/src/math_functions.py \
#    && chmod +x /home/catkin_ws/src/robotics_apps/line_follower/src/vision_functions.py \
#    && chmod +x /home/catkin_ws/src/robotics_apps/line_follower/src/vision_functions.py \
#    && chmod +x /home/catkin_ws/src/robotics_base/chassis_imu/src/imu_publisher.py \
#    && chmod +x /home/catkin_ws/src/robotics_base/chassis_move/src/chassis_move.py \
#    && chmod +x /home/catkin_ws/src/robotics_base/chassis_odom/src/odom_publisher.py \
#    && chmod +x /home/catkin_ws/src/robotics_base/chassis_servos/src/chassis_servos.py

# need ros-noetic-web-video-server and async_web_server_cpp but not available as debian packages yet
RUN cd /home/catkin_ws/src \
    && git clone https://github.com/GT-RAIL/async_web_server_cpp \
    && git clone https://github.com/RobotWebTools/web_video_server

# catkin_make and rosdep should ignore simulation packages
RUN touch /home/catkin_ws/src/robotics_configurations/alex_description/CATKIN_IGNORE
RUN touch /home/catkin_ws/src/robotics_configurations/alex_gazebo/CATKIN_IGNORE

RUN cd /home/catkin_ws \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# build with catkin_make and add to source command to .bashrc
RUN cd /home/catkin_ws/ \
    && source /ros_entrypoint.sh \
    && catkin_make \
    && echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc

# install ssh server for code deployment
RUN apt-get install -y openssh-server \
    && mkdir /var/run/sshd \
    && echo 'root:pi-top' | chpasswd \
    && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
    && echo "export VISIBLE=now" >> /etc/profile \
    && service ssh restart \
    && echo 'service ssh restart' >> ~/.bashrc

# comment out for development purposed to avoid having to run apt update again
# RUN rm -rf /var/lib/apt/lists/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]