# Poliautolab

Politecnico di Milano Duckietown Autolab.

Localization for our watchtower.

The node localization/watcher.py send a multicast UDP msg of type DuckPose in the group "my_position".
The node getmap_server provides the description of the map, again as a mUDP of group "my_map". 

To use install the <a href="https://github.com/duckietown/duckietown-shell">duckietown shell DTS</a> and run the following command:
```
dts devel build -f -H watchtower00.local
```
```
dts devel run -H watchtower.local
```

That's it. A UDP message is now being streamed and can be read from any device in the network.


To connect to the running ROS environment:
```
dts start_gui_tools watchtower00.local
```
Based on ROS, find the scripts inside packages/localization

If PUB_RECT is True a topic of type CompressedImage will be published with the rectified image and, if possible, the position of the duckiebot.

![rviz](assets/rviz_result.png)
