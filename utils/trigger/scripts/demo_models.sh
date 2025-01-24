#!/bin/bash

# Get the current username
username=$USER
echo "Current Username: $username"

# Navigate to the home directory
cd /home

#Launch the Azure ROS driver
export DISPLAY=:0
tmux new-session -d -s adl_session "export DISPLAY=:0 && roslaunch azure_kinect_ros_driver kinect_rgbd.launch"

# Bind Ctrl + C to kill the tmux session
tmux bind-key -n C-c kill-session

# Split the window horizontally to create a second pane for the audio recording script
tmux split-window -h -t adl_session

# Run the audio recording in the second tmux pane
tmux send-keys -t adl_session.1 "sleep 3; cd /home/$username/rehab_ws/src/rehab_core/utils/trigger/src && python3 demo_trigger.py" C-m

# Attach to the tmux session and focus on the second pane
tmux select-pane -t adl_session.1

# Attach to the tmux session (optional, if you want to see the output in real-time)
tmux attach-session -t adl_session
