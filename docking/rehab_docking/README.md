# rehab_docking
This is responsible for the auto docking of the robot.\
It follows four steps strategy:
* Navigate to a point in front of docking station using navigation stack
* Move towards Artag using the camera and stop at certain distance
* Rotate 180 degree
* Move backward untill the robot is docked (backwards motion is based upon the lidar's backward distance)

## Installation
### ar_track_alvar
This package detects the Artag. To install this, you need to go into your workspace using\
```cd catkin_ws/src/```\
install the arouco_ros\
```sudo apt-get install ros-noetic-arouco-ros```
### realsense_ros
```git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy```
### rehab_docking
```git clone https://github.com/UsamaArshad16/rehab_docking.git```

Build these packages\
```cd ..```\
```rosdep install --from-paths src --ignore-src -r -y```\
```catkin_make```\
```source devel/setup.bash```

## Run the demo
### 1. rehab_bringup
```roslaunch rehab_bringup rehab_bringup.launch```
### 2. rehab_navigation
open another terminal\
```roslaunch rehab_navigation rehab_navigation.launch```\
localize properly
### 3. realsense_ros
open another terminal\
```roslaunch realsense2_camera rs_camera.launch```
### 4. arouco_ros
You also need to specify your tag/marker size and topics in the arouco_ros.launch file.\
so you can set the args like this in launch file\
	```<arg name="markerId" default="16" />```\
	```<arg name="markerSize" default="0.1" />```\
	```<arg name="marker_frame" default="aruco_marker_frame" />```\
	```<arg name="output_frame" default="/camera_link" />```\
open another terminal\
```roslaunch rehab_bringup arouco_ros.launch```

### 5. rehab_docking
open another terminal\
```rosrun rehab_docking dock.py```

## Watch the Smoooth docking and enjoy...
