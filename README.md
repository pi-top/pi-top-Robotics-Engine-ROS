# pi-top Robotics Engine
Middleware based on ROS for building and programming pi-top Robotics.

pi-top's Robotics Engine runs inside a Ubuntu Docker Container and is currently based on Noetic (ROS 1) - this means 
Python 3 is supported out of the box. The Robotics Engine provides a number of services designed to abstract low-level
control of the hardware, enable modular software design and encourages re-use where possible.

# How to use
First, [download the latest pi-topOS](https://www.pi-top.com/products/os) and burn to an SD card. We recommend using 
a 32 GB SD card if your planning to do any development with docker containers as you can quickly run out of space
on a 16 GB one.

## Easy Method
The easiest way to try it out is to use one of our ready-to-go Docker images, but if you like compiling things from 
scratch then go to the next section.

First we need to install Docker:

```
cd ~/
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
```

(note: if you reboot before the next steps you can omit `sudo` from all the `docker` commands)

Then get the latest Docker image:

`sudo docker pull pitop/robotics-engine:latest`

Run it:

```
sudo docker run -it \
      -v /dev:/dev \
      --device-cgroup-rule='c 89:* rmw' \
      --device-cgroup-rule='c 81:* rmw' \
      -p 8022:22 -p 80:80 -p 8080:8080 -p 9090:9090 \
      --name pt-ros-dev \
      pitop/robotics-engine:latest
```

Once inside the container, start the robotics engine:

`roslaunch robotics_engine main.launch`

Open a new Raspberry Pi terminal and get another shell to your docker container by running:

`sudo docker exec -it pt-ros-dev bash`

`service nginx restart`

Now go to any device on the same network of the Raspberry Pi and put in the IP address into your browser, you should 
see a web controller interface and a FPV camera view.

## Compile from source

In a terminal run:

`git clone https://github.com/pi-top/pi-top-Robotics-Engine-ROS.git`

`cd pi-top-Robotics-Engine-ROS`

`chmod +x setup.sh`

`sudo bash setup.sh`

This will build from the base ROS noetic core image provided on Docker Hub and configure it to work with the pi-top 
Robotics Kit.

It will take around 30 minutes to finish + the time it takes you to download the noetic docker image - for future builds 
you won't need to download that image again. 

Once finished, the script will automatically run the container for you and you'll be in a container bash shell.

Now let's see if everything works.

With your pi-top [4] docked into an Expansion Plate (and ideally a robot attached to it, we recommend the Alex build 
configuration as it's been tested most), run the following:

`roslaunch robotics_engine main.launch`

Open a new Raspberry Pi terminal and get another shell to your docker container by running:

`docker exec -it pt-ros-dev bash`

Note: if you haven't logged out and in again (or rebooted) since docker was installed you'll need to use `sudo` to run
any docker commands.

Inside that shell:

`service nginx restart`

Now go to any device on the same network of the Raspberry Pi and put in the IP address into your browser, you should 
see a web controller interface and a FPV camera view.

### After rebooting

If you've rebooted and want to start the container again, run:

`docker ps -a`

To find the name of the container, it should be called `pt-ros-dev`, if not then substitute that in the following 
commands.

`docker start pt-ros-dev`

`docker exec -it pt-ros-dev bash`

You should now be inside the docker container in the `~/catkin_ws` directory.

# Running Gazebo simulations
 