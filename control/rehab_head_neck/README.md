![Version](https://img.shields.io/badge/MRR-XAVOR-yellow)
# Rehab Robot Head & Neck Integration


## Overview

### How to Run PAN-TILT For Testing From PC

1. Make sure roscore is running on Testing PC
2. Make sure the ESP on Hardware side is connected to same network and the IP of testing PC is set properly on ESP end. 
3. Run the following node on the testing PC:
   
   ```rosrun rosserial_python serial_node.py tcp 11411```
4. Run the following node for publishing the pan and tilt sample message to respective topic: 
   
   ```roslaunch rehab_head_neck_intg pan_tilt.launch```
   
   You can input number to several pan and tilt motions. And you can set parameters to tune any gesture.
   

    



### Published Topics

1. ```/tilt_angles ```

    Topic Type: 
    
    std_msgs::Int32MultiArray
    

2. ```/pan_angles```
   
    Topic Type: 
    
    std_msgs::Int32MultiArray
