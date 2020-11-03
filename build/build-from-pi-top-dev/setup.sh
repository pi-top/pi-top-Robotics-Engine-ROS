#!/bin/bash

# exit when any command fails
set -e
set -o xtrace

# image name:tag
IMAGE_NAME=pi-top-robotics-engine-dev:0.0.0

sudo apt update
if [ -x "$(command -v docker)" ]; then
    echo "Already installed"
    # add command to update if necessary
else
    echo "Install docker"
    # command
    curl -sSL https://get.docker.com | sh
    sudo usermod -aG docker pi
fi

# build docker image
sudo docker build -t $IMAGE_NAME .

# run and setup container
sudo docker run --privileged \
      -p 8022:22 -p 80:80 -p 8080:8080 -p 9090:9090 \
      $IMAGE_NAME \
      bash /home/container-build-from-noetic-setup.sh

