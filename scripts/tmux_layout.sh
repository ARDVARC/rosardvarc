#!/bin/bash

# https://unix.stackexchange.com/questions/553463/how-can-i-programmatically-get-this-layout-in-tmux

# Start roscore so that echoers have something to connect to
roscore &
sleep 3

# Open new tmux session. In first window, prepare to run the main FSW launch script
tmux -2 new-session -d -x "$(tput cols)" -y "$(tput lines)" -s "GCS" -n "FSW" 'echo "REMEMBER TO FIX LOCAL ORIGIN, UPDATE MAVROS RATES, AND RUN ROSBAG!"; bash'
sleep 2
tmux send-keys "roslaunch rosardvarc fsw_and_io.launch bagname:=CHANGE_THIS"

# In second window, add loggers for
# - Mission state
# - Mission state criteria
# - RGV states
# - Recent sightings
# - Bluetooth
# - UAS pose
tmux new-window -t "GCS:1" -n 'TOPICS' 'rostopic echo /state_machine/mission_states -c'

tmux split-window -t "GCS:1" -h -p 80 -d 'rostopic echo /state_machine/state_machine_criteria -c -w 5'
tmux split-window -t "GCS:1" -v -p 50 -d 'rostopic echo /camera/recent_rgv_sightings -c'
tmux select-pane  -t "GCS:1.{right}"
tmux split-window -t "GCS:1" -h -p 50 -d 'rostopic echo /estimation/estimated_rgv_states -c'
tmux split-window -t "GCS:1" -v -p 50 -d 'rostopic echo /bluetooth/az_els -c -w 5'
tmux select-pane  -t "GCS:1.{right}"
tmux split-window -t "GCS:1" -v -p 50 -d 'rostopic echo /mavros/local_position/pose -c -w 5'

# Add empty third window for commands
tmux new-window -t "GCS:2" -n 'COMMANDS'

# Attach to session
tmux select-window -t "GCS:0"
tmux -2 attach-session -t "GCS"
