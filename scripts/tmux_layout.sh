#!/bin/bash

# https://unix.stackexchange.com/questions/553463/how-can-i-programmatically-get-this-layout-in-tmux

# Open new session. In first window, prepare to run the main FSW launch script + MAVROS commands
tmux -2 new-session -d -x "$(tput cols)" -y "$(tput lines)" -s "GCS" -n "FSW" 'echo "REMEMBER TO FIX LOCAL ORIGIN, UPDATE MAVROS RATES, AND RUN ROSBAG!"; bash'
sleep 1
tmux send-keys "roslaunch rosardvarc fsw_and_io.launch"

# In second window, add loggers for
# - Mission state
# - Mission state criteria
# - RGV states
# - Recent sightings
# - Bluetooth
tmux new-window -t "GCS:1" -n 'Logs' 'rostopic echo /state_machine/mission_states -c'

tmux split-window -t "GCS:1" -h -p 80 -d 'rostopic echo /estimation/estimated_rgv_states -c -w 5'
tmux split-window -t "GCS:1" -v -p 50 -d 'rostopic echo /camera/recent_rgv_sightings -c'
tmux select-pane  -t "GCS:1.{right}"
tmux split-window -t "GCS:1" -h -p 50 -d 'rostopic echo  state_machine/state_machine_criteria -c'
tmux split-window -t "GCS:1" -v -p 50 -d 'rostopic echo /bluetooth/az_els -c -w 5'
tmux select-pane  -t "GCS:1.{right}"
tmux split-window -t "GCS:1" -v -p 50 'rostopic echo /mavros/local_position/pose -c -w 5'

# In third window, add rosbag command
tmux new-window -t "GCS:2" -n 'ROSBAG'
sleep 1
tmux send-keys "rosbag record -o ~/rosbags/ -b 0 --split --size=1024 --chunksize=128 --lz4 bluetooth/az_els mavros/local_position/pose estimation/direction_vectors_uas estimation/estimated_rgv_states state_machine/mission_states state_machine/state_machine_criteria state_machine/forced usb_cam/image_raw/compressed estimation/rgv_local_projections camera/recent_rgv_sightings"

# Add empty fourth window for commands
tmux new-window -t "GCS:3" -n 'COMMANDS'

# Attach to session
tmux select-window -t "GCS:0"
tmux -2 attach-session -t "GCS"