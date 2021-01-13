#!/bin/bash

# this setup file has to stay here so Docker context allows copy of src files, see -f on docker build command

# exit when any command fails
set -e
set -o xtrace

# image name:tag
IMAGE_NAME=pt-ros:0.0.0

#sudo apt update
#if [ -x "$(command -v docker)" ]; then
#    echo "Already installed"
#    # add command to update if necessary
#else
#    echo "Install docker"
#    # command
#    curl -fsSL https://get.docker.com -o get-docker.sh
#    sudo sh get-docker.sh
#    sudo usermod -aG docker pi
#fi


# Need to fix docker GPG key issue for Raspberry Pi
# This is not yet available in Debian's stable repos so download from here instead
#curl -fsSL http://ftp.us.debian.org/debian/pool/main/libs/libseccomp/libseccomp2_2.4.4-1~bpo10+1_armhf.deb \
#      -o libseccomp2_2.4.4-1~bpo10+1_armhf.deb
#sudo dpkg -i libseccomp2_2.4.4-1~bpo10+1_armhf.deb

# build docker image
sudo docker build -t $IMAGE_NAME -f build/build-from-noetic/Dockerfile .

# run container
sudo docker run -it \
      -v /dev:/dev --device-cgroup-rule='c 89:* rmw' --device-cgroup-rule='c 81:* rmw' \
      -p 8022:22 -p 80:80 -p 8080:8080 -p 9090:9090 \
      --name pt-ros-dev \
      $IMAGE_NAME