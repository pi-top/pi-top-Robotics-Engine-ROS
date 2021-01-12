FROM ros:noetic-ros-core

USER root

# Would usually install packages in Dockerfile at this point
# but can't get around GPG key issue with ros-noetic so using install script instead

# create catkin workspace
ENV CATKIN_WS /home/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# add robotics engine source files
ADD src $CATKIN_WS/src/

# add .bashrc entries
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
RUN echo 'cd /home/catkin_ws' >> ~/.bashrc

ADD build/build-from-noetic/scripts/container-setup.sh /home/
RUN ["chmod", "+x", "/home/container-setup.sh"]



