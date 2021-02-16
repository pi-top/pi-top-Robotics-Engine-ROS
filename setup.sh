#!/bin/bash

# exit when any command fails
set -e
set -o xtrace

# image name:tag
IMAGE_NAME=pt-ros-dev:0.0.1

sudo apt update
if [ -x "$(command -v docker)" ]; then
    echo "Already installed"
    # add command to update if necessary
else
    echo "Install docker"
    # command
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker pi
fi

# Need to fix docker GPG key issue for Raspberry Pi
# This is not yet available in Debian's stable repos so download from here instead
curl -fsSL http://ftp.us.debian.org/debian/pool/main/libs/libseccomp/libseccomp2_2.4.4-1~bpo10+1_armhf.deb \
      -o libseccomp2_2.4.4-1~bpo10+1_armhf.deb
sudo dpkg -i libseccomp2_2.4.4-1~bpo10+1_armhf.deb

# build docker image
sudo docker build -t $IMAGE_NAME -f Dockerfile .

# run container
sudo docker run -it \
      -v /dev:/dev \
      --device-cgroup-rule='c 89:* rmw' \
      --device-cgroup-rule='c 81:* rmw' \
      -p 8022:22 -p 80:80 -p 8080:8080 -p 9090:9090 \
      --name pt-ros-dev \
      $IMAGE_NAME

# argument meanings, in order:
# run in interactive mode
# Add access to udev information so docker containers can get more info on your usb devices
# Add access to /dev/i2c-* devices
# Add access to /dev/video* devices
# SSH port, web server port, web_video_server port, websocket port
# container name
# image name to use
