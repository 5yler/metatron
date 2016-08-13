#!/bin/bash

# post_flashing_tk1_install.sh
# Bash script to install CUDA, OpenCV, ROS, relevant ROS packages, etc.
#
# @author   Syler Wagner     <syler@mit.edu>
# @author   Chris Desnoyers  <cjdesno@mit.edu>
# 
# @date     2016-02-15       creation

sudo apt-mark hold xserver-xorg-core # (to stop the GUI from getting fucked later)

sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted

sudo apt-get -y --force-yes update

sudo apt-get -y --force-yes install nano # Install Nano
echo "export EDITOR='nano' # use nano as default text editor"  >> ~/.bashrc

sudo apt-get -y --force-yes install samba # for networked drives
sudo apt-get -y --force-yes install tmux # tmux for splitting terminal into panes

sudo ntpdate ntp.ubuntu.com # set ubuntu date server
# This will ensure that your machine polls the ubuntu server to get the right time and date for your timezone. 

#####################################################################################
## Get CUDA Toolkit #################################################################

wget http://developer.download.nvidia.com/compute/cuda/6_5/rel/installers/cuda-repo-l4t-r21.2-6-5-prod_6.5-34_armhf.deb
# Install the CUDA repo metadata that you downloaded manually for L4T
sudo dpkg -i cuda-repo-l4t-r21.2-6-5-prod_6.5-34_armhf.deb
# Download & install the actual CUDA Toolkit including the OpenGL toolkit from NVIDIA. (It only downloads around 15MB)
sudo apt-get -y --force-yes update

# Install "cuda-toolkit-6-0" if you downloaded CUDA 6.0, or "cuda-toolkit-6-5" if you downloaded CUDA 6.5, etc.
sudo apt-get -y --force-yes install cuda-toolkit-6-5
# Add yourself to the "video" group to allow access to the GPU
sudo usermod -a -G video $USER

# Details: http://elinux.org/Jetson/Installing_CUDA

# Add the 32-bit CUDA paths to your .bashrc login script, and start using it in your current console:

echo "# Add CUDA bin & library paths:" >> ~/.bashrc
echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc

# Verify that the CUDA Toolkit is installed on your device:
nvcc -V

#####################################################################################
## Get OpenCV4Tegra #################################################################

wget http://developer.download.nvidia.com/embedded/OpenCV/L4T_21.2/libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb
sudo dpkg -i libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb
sudo apt-get -y --force-yes update
sudo apt-get -y --force-yes install libopencv4tegra libopencv4tegra-dev

# Details: http://elinux.org/Jetson/Installing_OpenCV

#####################################################################################
## Get ROS Indigo ###################################################################

# Details: https://github.com/jetsonhacks/installROS/blob/master/installROS.sh

# Setup Locale
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
# Installation
sudo apt-get -y --force-yes update
sudo apt-get -y --force-yes install ros-indigo-ros-base -y

# Initialize rosdep
sudo apt-get -y --force-yes install python-rosdep -y
sudo rosdep init
# To find available packages, use:
rosdep update
# Environment Setup
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Install rosinstall
sudo apt-get -y --force-yes install python-rosinstall -y

#####################################################################################
## Get OpenNI 2 for RGBD Sensors ####################################################

sudo apt-get -y --force-yes install ros-indigo-rgbd-launch ros-indigo-openni2-camera ros-indigo-openni2-launch
sudo apt-get -y --force-yes install ros-indigo-rqt ros-indigo-rqt-common-plugins ros-indigo-rqt-robot-plugins

#####################################################################################
## Install Arduino IDE  #############################################################

sudo apt-get -y --force-yes install arduino arduino-core

# Configure ROS Serial Communication With Arduino 
sudo apt-get -y --force-yes install ros-indigo-rosserial-arduino
sudo apt-get -y --force-yes install ros-indigo-rosserial

#####################################################################################
## Configure Neato XV Lidar #########################################################

sudo apt-get -y --force-yes install ros-indigo-xv-11-laser-driver 

# Details: http://wiki.ros.org/xv_11_laser_driver

#####################################################################################
## RViz Setup #######################################################################

#Rviz on the jetson is broken by default because of pcre3, a perl regex library that 
# doesn’t want to run on ARM (see https://github.com/ros/robot_model/issues/110). 
# The fix:

wget http://launchpadlibrarian.net/182261128/libpcre3_8.35-3ubuntu1_armhf.deb
wget http://launchpadlibrarian.net/182261131/libpcrecpp0_8.35-3ubuntu1_armhf.deb
wget http://launchpadlibrarian.net/182261132/libpcre3-dev_8.35-3ubuntu1_armhf.deb
sudo dpkg -i libpcre3_8.35-3ubuntu1_armhf.deb
sudo dpkg -i libpcrecpp0_8.35-3ubuntu1_armhf.deb
sudo dpkg -i libpcre3-dev_8.35-3ubuntu1_armhf.deb

# Installs ARM version of relevant library.
# I also ran apt-mark hold on them, not sure if that was necessary.
sudo apt-mark hold libpcre3
sudo apt-mark hold libpcre3-dev
sudo apt-mark hold libpcrecpp0

#####################################################################################
## GPS and Localization/Navigation ##################################################

sudo apt-get -y --force-yes install ros-indigo-nmea-navsat-driver

sudo apt-get -y --force-yes install ros-indigo-gps-common
sudo apt-get -y --force-yes install ros-indigo-robot-localization
sudo apt-get -y --force-yes install ros-indigo-navigation

# Hector SLAM
sudo apt-get -y --force-yes install ros-indigo-hector-slam
# URDF Model
sudo apt-get -y --force-yes install liburdfdom-tools # need this for check_urdf

#####################################################################################
## IMU Configuration (Partial) ######################################################

sudo apt-get -y --force-yes install cmake-curses-gui # for cmake configuration of IMU

# IMU visualizer for demo
sudo apt-get -y --force-yes install ros-indigo-razor-imu-9dof
sudo apt-get -y --force-yes install python-visual
sudo apt-get -y --force-yes install python-wxtools

#####################################################################################
## Create Catkin Workspace ##########################################################

# Details: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

# source environment
source /opt/ros/indigo/setup.bash

#  create a catkin workspace:
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# "build" the empty workspace:
cd ~/catkin_ws/
catkin_make

# source your new setup.*sh file:
source devel/setup.bash

#####################################################################################
## Update Everything Again ################################################################

sudo apt-get -y --force-yes update        # Fetches the list of available updates
sudo apt-get -y --force-yes upgrade       # Strictly upgrades the current packages
sudo apt-get -y --force-yes dist-upgrade  # Installs updates (new ones)

#####################################################################################
## Additional Setup Notes ###########################################################

## The following are not part of the shell script, and must be done manually.

# Change Hostname ###################################################################
# Replace tegra-ubuntu with <newhostname> in the files:
# sudo nano /etc/hostname
# sudo nano /etc/hosts

# Set Up Networked Drive ############################################################

# IMU Configuration #################################################################

# Clone Gigatron GitHub Repository ##################################################

# Enable Dynamic DNS ################################################################

# SSH Keys ##########################################################################

