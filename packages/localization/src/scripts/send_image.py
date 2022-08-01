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

# Duckie stuff
from dt_communication_utils import DTCommunicationGroup

# from std_msgs.msg import String
group = DTCommunicationGroup('image', CompressedImage)
groupinfo = DTCommunicationGroup('camera_info', CameraInfo)

VERBOSE=False


def _cb(ros_data, dt_publish):
    """
    Callback function for subscribed topic.
    Get image and extract position and orientation of Duckiebot.
    Publish duckiebot position and orientation on /watchtower00/localization.

    :param ros_data: received image

    :type ros_data: sensor_msgs.msg.CompressedImage
    """
    dt_publish.publish(ros_data)


def main():
    rospy.init_node("send_image")
    print("[Sender]: Starting...")
    dt_publish_img = group.Publisher()
    dt_publish_info = groupinfo.Publisher()
    rospy.Subscriber("/watchtower00/camera_node/image/compressed", CompressedImage, lambda ros_data: _cb(ros_data, dt_publish_img), queue_size=1, buff_size=2**24)
    rospy.Subscriber("/watchtower00/camera_node/camera_info",  CameraInfo, lambda ros_data: _cb(ros_data, dt_publish_info), queue_size=1)

if __name__ == '__main__':
    main()