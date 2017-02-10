# `metatron`
Autonomous Power Racing Series Racecar - ROS Metapackage

## `gigatron_hardware`
The ROS wrapper code that interfaces with the Arduino lives here. So do sensor launch files and hardware config files. See also: the [`gigabug`](http://github.com/5yler/gigabug/) repository with Arduino code for the car's motor control.

## `gigatron`
Main race code. Could be cleaned up a bit more.

## `gigatron_hardware`
The ROS wrapper code that interfaces with the Arduino lives here. So do sensor launch files and hardware config files.

## `gigatron_nav`
Localization and navigation code and launch files. 

## `scan_filters`
Filters for `sensor_msgs/LaserScan` messages to remove noise from sunlight and filter scans based on angle.

## `metatron` 
ISMETA* and or some reason this nested folder structure was required to make it a metapackage. Don't ask.

# Usage Instructions
Clone the entire metapackage into your catkin workspace. Run `catkin_make`.

*I'm So Meta, Even This Acronym
