# Persistent USB Device Naming

We can use `udev` to make serial ports for devices unique and persistent. Symlink the `99-usb-serial-device.rules` file into `/etc/udev/rules.d` by running:

```
rosrun gigatron_hardware init_udev_rules
```
The result is everytime you plug in a recognized device, the system will symlink it appropriately. No more `/dev/ttyACM0` and `dev/ttyACM1`naming conflicts!

```
15:04 board /etc/udev/rules.d ls -l /dev/*
0 lrwxrwxrwx 1 root 7 Aug 18 15:04 /dev/rplidar -> ttyUSB0
0 lrwxrwxrwx 1 root 7 Aug 18 15:04 /dev/xvlidar -> ttyACM0
0 lrwxrwxrwx 1 root 7 Aug 18 15:04 /dev/arduino -> ttyACM1
```
