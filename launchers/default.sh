#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# roscore &
# sleep 2
# printf "Rosbag play\n\n"
# sleep 2
# rosbag play -q -l $DT_REPO_PATH/bags/2022-07-07-13-19-43.bag --clock &
# sleep 2
rosrun localization getmap_server.py &
printf "Map server runnning...\n\n"
sleep 2
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map odom 10 &
printf "Tf done\n\n"
sleep 5
# Takes some time to start on the raspberry...
sleep 30
rosservice call /get_map
printf "Get map called, param server should be updated\n\n"
sleep 2
dt-exec rosrun localization watcher.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
