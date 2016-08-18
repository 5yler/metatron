# Gigatron Hardware Launch Files

## `sensor_transform.launch`
Launch file for publishing transform from `base_link` to individual sensor link.

#### Example usage:
```
roslaunch gigatron_hardware sensor_transform.launch car:=kilotron sensor:=imu
```
#### Prerequisites:
- The static transform must be defined in `gigatron_hardware/config/<CAR_NAME>/sensor_links/<SENSOR>_link.yaml`
- You must have this [forked version of the `tf` package](https://github.com/orzechow/geometry/tree/indigo-devel/tf), which supports parameter loading in the `static_transform_publisher` node
