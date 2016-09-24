#!/bin/bash

# post_flashing_tx1_install.sh
# Bash script to install ROS, relevant ROS packages, etc. on Jetson TX1
#
# @author   Syler Wagner     <syler@mit.edu>
# @author   Chris Desnoyers  <cjdesno@mit.edu>
# 
# @date     2016-09-23       creation

sudo apt-get -y --force-yes install nano # Install Nano
echo "export EDITOR='nano' # use nano as default text editor"  >> ~/.bashrc

sudo apt-get -y --force-yes install samba # for networked drives
sudo apt-get -y --force-yes install tmux # tmux for splitting terminal into panes

sudo ntpdate ntp.ubuntu.com # set ubuntu date server
# This will ensure that your machine polls the ubuntu server to get the right time and date for your timezone. 

#####################################################################################
## Get OpenNI 2 for RGBD Sensors ####################################################

sudo apt-get -y --force-yes install ros-indigo-rgbd-launch ros-indigo-openni2-camera ros-indigo-openni2-launch

#####################################################################################
## Install Arduino IDE  #############################################################

sudo apt-get -y --force-yes install arduino arduino-core

# Configure ROS Serial Communication With Arduino 
sudo apt-get -y --force-yes install ros-indigo-rosserial-arduino
sudo apt-get -y --force-yes install ros-indigo-rosserial

#####################################################################################
## Configure LIDAR sensors ##########################################################

sudo apt-get -y --force-yes install ros-indigo-xv-11-laser-driver ros-indigo-rplidar-ros
sudo apt-get -y --force-yes install ros-indigo-laser-pipeline ros-indigo-pointcloud-to-laserscan ros-indigo-depthimage-to-laserscan 


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

sudo apt-get -y --force-yes install ros-indigo-rviz-* ros-indigo-rqt-*

#####################################################################################
## GPS and Localization/Navigation ##################################################

sudo apt-get -y --force-yes install ros-indigo-nmea-navsat-driver

sudo apt-get -y --force-yes install ros-indigo-gps-common
sudo apt-get -y --force-yes install ros-indigo-robot-localization
sudo apt-get -y --force-yes install ros-indigo-navigation ros-indigo-teb-local-planner

# SLAM
sudo apt-get -y --force-yes install ros-indigo-hector-slam ros-indigo-gmapping ros-indigo-rtabmap-ros


#####################################################################################
## Hardware/URDF Models #############################################################

sudo apt-get -y --force-yes install liburdfdom-tools ros-indigo-xacro ros-indigo-robot-state-publisher # urdf

#####################################################################################
## IMU Configuration (Partial) ######################################################

sudo apt-get -y --force-yes install cmake-curses-gui # for cmake configuration of IMU

# IMU visualizer for demo
sudo apt-get -y --force-yes install ros-indigo-razor-imu-9dof
sudo apt-get -y --force-yes install python-visual
sudo apt-get -y --force-yes install python-wxtools

sudo apt-get -y --force-yes install libi2c-dev i2c-tools

# add udev rule so i2c devices aren't only owned by root
sudo sh -c 'echo "KERNEL==\"i2c-[0-7]\",MODE=\"0666\"" > /etc/udev/rules.d/90-i2c.rules'

cd ~
git clone git@github.com:jetsonhacks/RTIMULib.git
cd RTIMULib/Linux
mkcd build
mkdir build
cd build/
cmake ..
make -j4
sudo make install
sudo ldconfig
sudo rm -rf ~/RTIMULib

#####################################################################################
## Update Everything Again ################################################################

sudo apt-get update        # Fetches the list of available updates
sudo apt-get upgrade       # Strictly upgrades the current packages

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

