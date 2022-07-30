# Poliautolab

Politecnico di Milano Duckietown Autolab.

Localization for our watchtower.

The node localization/watcher.py send a multicast UDP msg of type DuckPose in the group "my_position".
The node getmap_server provides the description of the map, again as a mUDP of group "my_map". 

To use install the duckietown shell DTS and run the following command:
```
dts devel build -f
```
```
dts devel run
```
To open rviz open another terminal using
```
dts start_gui_tools
```
Based on ROS, find the scripts inside packages/localization

If PUB_RECT is True a topic of type CompressedImage will be published with the rectified image and, if possible, the position of the duckiebot.

![rviz](assets/rviz_result.png)
