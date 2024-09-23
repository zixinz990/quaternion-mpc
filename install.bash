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

# Install OSQP
cd /tmp
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
git checkout v0.6.3
git submodule update --recursive
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install

# Install OSQP-Eigen
cd /tmp
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
git checkout v0.8.0
mkdir build
cd build
cmake ..
make
sudo make install

cd ~
sudo apt-get install ros-noetic-joy -y