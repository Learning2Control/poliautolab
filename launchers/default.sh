#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
roscore &
sleep 5
printf "Rosbag play\n\n"
rosbag play -q -l $DT_REPO_PATH/bags/2022-05-16-16-18-59.bag &
sleep 5
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map odom 10 &
printf "Tf done\n\n"
# sleep 5
dt-exec rosrun localization getmap_server.py &
printf "Map server runnning...\n\n"
sleep 2
rosservice call /get_map
printf "Get map called, param server should be updated\n\n"
sleep 2
dt-exec rosrun localization watcher.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
