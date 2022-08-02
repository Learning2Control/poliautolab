#!/usr/bin/env python

"""Detects duckiebots from the watchtower using their unique colors.

Publish on watchtower00/localization a DuckPose with the car coordinates.
Publish using dt_communication.
"""
__author__ =  'Giulio Vaccari <giulio.vaccari at mail.polimi.it>'
__version__=  '0.1'
__license__ = 'MIT'

# Ros libraries
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage, CameraInfo

import numpy as np
import cv2

# Duckie stuff
from dt_communication_utils import DTCommunicationGroup

# from std_msgs.msg import String
group = DTCommunicationGroup('image', CompressedImage)
groupinfo = DTCommunicationGroup('camera_info', CameraInfo)

# LCM detected that large packets are being received, but the kernel UDP
# receive buffer is very small.  The possibility of dropping packets due to
# insufficient buffer space is very high.

# For more information, visit:
#    http://lcm-proj.github.io/multicast_setup.html

def _imgcb(ros_data):
    np_arr = np.frombuffer(ros_data.data, 'u1')
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def main():
    rospy.init_node("send_image")
    print("[Sender]: Starting...")
    dt_publish_img = group.Publisher()
    dt_publish_info = groupinfo.Publisher()
    # rospy.Subscriber("/watchtower00/camera_node/image/compressed", CompressedImage, lambda ros_data: dt_publish_img.publish(ros_data), queue_size=1, buff_size=2**24)
    rospy.Subscriber("/watchtower00/camera_node/camera_info",  CameraInfo, lambda ros_data: dt_publish_info.publish(ros_data), queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()