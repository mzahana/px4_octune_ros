#!/bin/bash

# Author: Mohamed Abdelkader, mohamedashraf123@gmail.com
# This script installs the px4_octune_ros dependencies
# Tested on Ubuntu 18 + ROS melodic

echo "Installing px4_octune_ros dependencies ..." && echo
sleep 1

if [ -z ${GIT_TOKEN} ]; then
    CLONE_OCTUNE="git clone https://github.com/mzahana/octune.git"
else
    CLONE_OCTUNE="git clone https://${GIT_TOKEN}@github.com/mzahana/octune.git"
fi

if [-z ${SUDO_PASS} ]; then
    sudo apt-get update
else
    echo $SUDO_PASS | sudo -S apt-get update
fi


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
    $CLONE_OCTUNE
else
    echo "octune package already exists in $HOME/src/octune. Pulling latest updates..." && echo
    cd $HOME/src/octune
    git pull
fi
echo && echo "Checking out python2 branch..." echo
cd $HOME/src/octune && git checkout python2

# Make sure we have python3-pip installed

#echo "Installing python-pip..." && echo
#sleep 1
#sudo apt install python3-pip -y
#sudo apt install python-pip -y

# This is to solve installation issues related to pip (Python 2.7)
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
python get-pip.py --force-reinstall

if [-z ${SUDO_PASS} ]; then
    sudo pip uninstall -y numpy
else
    echo $SUDO_PASS | sudo -S pip uninstall -y numpy
fi

pip install numpy
pip install matplotlib --user
pip install scipy --user

# Install octune
cd $HOME/src/octune
#python3 setup.py develop --user
python setup.py develop --user
