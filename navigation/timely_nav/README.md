# timely_nav
The timely_nav package is a ROS package that allows a robot to navigate to different locations at specific times. The package provides a node that subscribes to the system time and publishes a message on a ROS topic to navigate the robot to a specific location when a specific time is reached.

## Installation
1. Clone the timely_nav package to your ROS workspace:\
 ```cd ~/catkin_ws/src```\
 ```git clone https://github.com/UsamaArshad16/timely_nav.git```
 
2. Build the package:\
 ```cd ~/catkin_ws```\
 ```catkin_make```

## Usage
To use the timely_nav package, you need to launch the timely_nav node using the provided launch file.

 ```roslaunch timely_nav timely_nav.launch```
 
This will start the timely_nav node, which will listen for system time and publish messages on the /gotogoal topic to navigate the robot to a specific location when a specific time is reached.
