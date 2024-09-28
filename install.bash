#!/bin/bash

# Create a folder for dependencies
cd ~
mkdir -p Documents/Dev/dependencies

# Install CMake 3.24.3
cd /tmp
sudo apt-get -y install wget libtool
wget https://github.com/Kitware/CMake/releases/download/v3.24.3/cmake-3.24.3.tar.gz
tar -zxvf cmake-3.24.3.tar.gz
cd /tmp/cmake-3.24.3
./bootstrap
make
sudo make install

# Install lcm
cd ~/Documents/Dev/dependencies
git clone https://github.com/lcm-proj/lcm.git
cd lcm
git checkout v1.4.0
mkdir build
cd build
cmake ..
make
sudo make install

# Install unitree_legged_sdk
cd ~/Documents/Dev/dependencies
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
cd unitree_legged_sdk
git checkout v3.8.0
mkdir build
cd build
cmake ..
make
echo "export UNITREE_LEGGED_SDK_PATH=~/Documents/Dev/dependencies/unitree_legged_sdk" >> ~/.bashrc
source ~/.bashrc

# Install CasADi
cd /tmp
git clone https://github.com/casadi/casadi.git -b main casadi
cd casadi
mkdir build
cd build
sudo apt-get install swig
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON ..
make
sudo make install

# Install gram_savitzky_golay
cd /tmp
git clone https://github.com/arntanguy/gram_savitzky_golay.git
cd gram_savitzky_golay
git submodule init
git submodule update --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install

# Install ROS-joy
cd ~
sudo apt-get install ros-noetic-joy -y