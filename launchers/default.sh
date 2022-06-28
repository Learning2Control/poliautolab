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
echo "Rosbag play"
rosbag play -q -l $DT_REPO_PATH/bags/2022-05-16-16-18-59.bag &
sleep 5
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map odom 10 &
echo "Tf done"
# sleep 5
dt-exec rosrun localization getmap_server.py &
echo "Map server runnning..." &
sleep 5
dt-exec rosrun localization watcher.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
