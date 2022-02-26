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

# If the src directory of OCTUNE pkg is not supplied as env variable,
# use $HOME/src
if [-z ${OCTUNE_DIR} ]; then
    
    if [ ! -d "$HOME/src" ]; then
        echo "Creating $HOME/src" && echo
        mkdir -p $HOME/src
    fi
    OCTUNE_DIR=$HOME/src
fi

# Clone OCTUNE package
if [ ! -d "$OCTUNE_DIR/octune" ]; then
    cd $OCTUNE_DIR
    $CLONE_OCTUNE
else
    echo "octune package already exists in $OCTUNE_DIR/octune. Pulling latest updates..." && echo
    cd $OCTUNE_DIR/octune
    git pull
fi
echo && echo "Checking out python2 branch..." echo
cd $OCTUNE_DIR/octune && git checkout python2

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

pip install numpy --user
pip install matplotlib --user
pip install scipy --user
pip install pandas --user

# Install octune
cd $OCTUNE_DIR/octune
#python3 setup.py develop --user
python setup.py develop --user
