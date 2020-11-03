FROM ros:noetic-ros-core

USER root

# Would usually install packages in Dockerfile at this point
# but can't get around GPG key issue with ros-noetic, use install script instead

# create catkin workspace
ENV CATKIN_WS /home/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
# add ros-rover files
ADD ros-rover $CATKIN_WS/src/ros-rover
ADD ../../src $CATKIN_WS/src/
ENV PYTHON_LIBS $CATKIN_WS/src/robotics_base/hardware_interface

# add .bashrc entries
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
RUN echo 'cd /home/catkin_ws' >> ~/.bashrc
RUN echo 'export PYTHONPATH="${PYTHONPATH}:$PYTHON_LIBS"' >> ~/.bashrc

ADD script/container-setup.sh /home/
RUN ["chmod", "+x", "/home/container-setup.sh"]
RUN ["/bin/bash", "/home/container-setup.sh"]



