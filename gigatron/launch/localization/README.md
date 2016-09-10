
# Localization on a Rosbag

This is an example of running AMCL localization on rosbag data. Start playing a bag:
```
rosbag play canny_hallway_fixed_laser_drive_2016-09-08-01-22-18.bag -s 120 --clock
```
The above command starts playing the bag 120 seconds in, and publishes ROS time information along with the messages.

## Running `amcl` node

Launch `amcl` with the argument `using_bag:=true` (this sets the `use_sim_time` ROS parameter to true) on the `canny_loop` map with an initial pose of `(x, y, yaw) = (0, 0, pi/4)`:
```
roslaunch gigatron amcl.launch x:=0 y:=0 yaw:=0.785 using_bag:=true map_server:=true map:=canny_loop
```
## Displaying Localization Results

Launch RViz as configured for map and localization viewing:
```
roslaunch gigatron rviz.launch config:=car_map
```
## Resetting Pose Estimate
### Local Reset

To reset the pose estimate, click on "2D Pose Estimate" at the top of the RViz display, then click and drag the green arrow on the map to select the position and orientation of the robot.
![image](https://cloud.githubusercontent.com/assets/10868851/18414002/83d1c0ac-7788-11e6-9905-1e7c2eaff8f7.png)

## Global Reset
To globally reset the localization particles across all the unoccupied space on the map, run:
```
rosservice call global_localization
```
![image](https://cloud.githubusercontent.com/assets/10868851/18414018/36d0291e-7789-11e6-8ea8-7e36ad81a6e7.png)

After a while, the particles will begin to cluster:

![image](https://cloud.githubusercontent.com/assets/10868851/18414023/6bcf5f5e-7789-11e6-9f76-d87186da463c.png)


![image](https://cloud.githubusercontent.com/assets/10868851/18414028/8d7347a6-7789-11e6-980d-c19eec518a6a.png)
