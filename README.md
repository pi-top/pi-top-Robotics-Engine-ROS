# pi-top Robotics Engine
Middleware based on ROS for building and programming pi-top Robotics.

pi-top's Robotics Engine runs inside a Ubuntu Docker Container and is currently based on Noetic (ROS 1) - this means 
Python 3 is supported out of the box. The Robotics Engine provides a number of services designed to abstract low-level
control of the hardware, enable modular software design and encourages re-use where possible.

# How to use
First, [download the latest pi-topOS](https://www.pi-top.com/products/os) and burn to an SD card. We recommend using 
a 32 GB SD card if your planning to do any development with docker containers as you can quickly run out of space
on a 16 GB one.

In a terminal run:

`git clone https://github.com/pi-top/pi-top-Robotics-Engine-ROS.git`

`cd pi-top-Robotics-Engine-ROS`

`chmod +x build-from-noetic-setup.sh`

`sudo bash build-from-noetic-setup.sh`

This will build from the base ROS noetic core image provided on Docker Hub and configure it to work with the pi-top 
Robotics Kit.

It will take around 30 minutes to finish + the time it takes you to download the noetic docker image - for future builds 
you won't need to download that image again. 

Once finished, you'll want to reboot so that you can use the `docker` command without having to use `sudo`.

After that, run this to see the docker container name:

`docker ps -a`

Then run:

`docker start YOUR_CONTAINER_NAME`

`docker exec -it YOUR_CONTAINER_NAME bash`

If all went well, you'll now be inside the docker container inside `~/catkin_ws`. 

Now let's see if everything works.

With your pi-top [4] docked into an Expansion Plate (and ideally a robot attached to it, we recommend the Alex build 
configuration), run the following:

`roslaunch robotics_engine main.launch`

Open a new Raspberry Pi terminal and get another shell to your docker container by running:

`docker exec -it YOUR_CONTAINER_NAME bash`

Inside that shell:

`service nginx restart`
