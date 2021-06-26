#!/bin/bash

# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com
# This script installs the px4_octune_ros dependencies
# Tested on Ubuntu 18 + ROS melodic

echo "Installing px4_octune_ros dependencies ..." && echo
sleep 1

sudo apt-get update

# We need a src folder in the HOMe folder
if [ ! -d "$HOME/src" ]; then
    echo "Creating $HOME/src" && echo
    sleep 1
    mkdir -p $HOME/src
fi
cd $HOME/src

# Clone OCTUNE package
if [ ! -d "$HOME/src/octune" ]; then
    cd $HOME/src
    git clone https://github.com/mzahana/octune.git
else
    echo "octune package already exists in $HOME/src/octune. Pulling latest updates..." && echo
    cd $HOME/src/octune
    git pull
fi

# Make sure we have python3-pip installed
echo "Installing python3-pip..." && echo
sleep 1
sudo apt install python3-pip -y

# Install octune
cd $HOME/src/octune
python3 setup.py develop --user

echo "OCTUNE package is installed." && echo
sleep 1