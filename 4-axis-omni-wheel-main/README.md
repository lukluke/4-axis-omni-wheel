# EE3070_2122_4-axis-omni-wheel

Written by EE3070 2021-2022 Semester B GP5




## Basic info

The ROS package is responsible for subscribing joystick data, converting and publishing RPM speeds to the Arduino program.
The `grp5_node` will create 4 seperate `Float32` channels of name
```
whlspd1
whlspd2
whlspd3
whlspd4
```
of which is subscribed by `Rm_motor_speed_control`.

## Build

```
# While in ../3070grp5/ros_workspace
colcon build
```

## Launch

Simply run `launch_script.bash`.
The script will launch `joy_node`, `grp5_node` and initialize the DDS-Agent needed for the arduino.
