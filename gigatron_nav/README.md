# Car Navigation Stack

The [`nav.launch`](/gigatron_nav/launch/nav.launch) file launches:
* `map_server` with a specified map
* `amcl` localization
* [`move_base`](http://wiki.ros.org/move_base) with car-specific configuration


The configuration files for `move_base` in the [`gigatron_nav/config`](/gigatron_nav/config/) directory specify [`base_local_planner`](wiki.ros.org/base_local_planner) parameters, and both local and global [2D costmap](wiki.ros.org/costmap_2d) configuration.

### Example `nav.launch` Usage

```
roslaunch gigatron_nav nav.launch car:=gigatron map:=canny_loop 
```

## Viewing Local Costmap

After launching `nav.launch`:
```
roslaunch gigatron rviz.launch config:=car_costmap
```

Here is the local costmap overlaid with the static map, and they line up well:

![image](https://cloud.githubusercontent.com/assets/10868851/18414915/4b6f1aac-77ab-11e6-9461-af1b66140ef8.png)

Even if localization drifts, the local costmap maintains an accurate representation of the immediate surroundings.

![image](https://cloud.githubusercontent.com/assets/10868851/18414922/843d6ee2-77ab-11e6-98b3-63df5737278a.png)

