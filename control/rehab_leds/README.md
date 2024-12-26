# rehab_leds
The rehab_leds package is a ROS package that allows you to control the state of 3 LEDs (red, green, off) through ROS parameters.

## Installation
```cd catkin_ws/src/```\
```git clone https://github.com/Xavorcorp/rehab_leds.git```\
```cd ..```\
```rosdep install --from-paths src --ignore-src -r -y```\
```catkin_make```\
```source devel/setup.bash```

## Run the demo
### 1. ros master
```roscore```
### 2. rosserial python
open another terminal\
```rosrun rosserial_python serial_node.py tcp 11411```
### 3. led control node
open another terminal\
```roslaunch rehab_leds rehab_leds.launch```
### 4. set the state of leds
open another terminal\
```rosparam set led_1_set red```
