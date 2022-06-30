#!/usr/bin/env python
"""Detects duckiebots from the watchtower using their unique colors.

Publish on watchtower00/localization a PointStamped with the car coordinates.

Edited version of the original example by Simon Halle.
"""
__author__ =  'Giulio Vaccari <giulio.vaccari at mail.polimi.it>'
__version__=  '0.1'
__license__ = 'MIT'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import tf

# Ros Messages
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from localization.msg import DuckPose

VERBOSE=False
PLOT=False

def get_car(img):
    """
    Extract the car from the image.

    :param img: image
    :return: front coord, left coord, theta
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hsv_color1_blue = np.array([50, 200, 50])
    hsv_color2_blue = np.array([100, 300, 300])
    mask_blue = cv2.inRange(img_hsv, hsv_color1_blue, hsv_color2_blue)

    hsv_color1_pink = np.array([100, 50, 200])
    hsv_color2_pink = np.array([200, 200, 250])
    mask_pink = cv2.inRange(img_hsv, hsv_color1_pink, hsv_color2_pink)
    
    back_coo = np.argwhere(mask_pink==255).mean(axis=0)[::-1]
    front_coo = np.argwhere(mask_blue==255).mean(axis=0)[::-1]
    
    x_center = (front_coo[0] + back_coo[0])/2
    y_center = (front_coo[1] + back_coo[1])/2
    angle = np.arctan2(-front_coo[1]+back_coo[1], -front_coo[0]+back_coo[0])
    
    return x_center, y_center, angle

class ImageFeature:

    def __init__(self):
        """
        Initialize the ImageFeature class.
        """

        self.coordinates_pub = rospy.Publisher("/watchtower00/localization", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/watchtower00/camera_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/image/compressed")


    def callback(self, ros_data):
        """
        Callback function for subscribed topic.
        Get image and extract position and orientation of Duckiebot.
        Publish duckiebot position and orientation on /watchtower00/localization.

        :param ros_data: received image

        :type ros_data: sensor_msgs.msg.CompressedImage
        """
        if VERBOSE :
            print(f'received image of type: "{ros_data.format}"' )

        #### direct conversion to CV2 ####
        np_arr = np.frombuffer(ros_data.data, 'u1')
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Img has origin on top left, after the interpolation it will be rotated of 90 degrees, need to prevent that
        image_np = cv2.flip(image_np, 0)

        localized = False

        try:
            x, y, theta = get_car(image_np)
            if PLOT:
                print(x, y, theta)
                cv2.circle(image_np, [int(x),int(y)], 20, [0,0,255], -1)
                cv2.arrowedLine(image_np, [int(x),int(y)], [int(x-50*np.cos(theta)),int(y-50*np.sin(theta))], [0,255,0], 5)
            localized = True
        except ValueError:
            print("No lines found.")

        if PLOT:
            cv2.imshow('cv_img', image_np)
            cv2.waitKey(2)

        # Rotate and remove offset
        if rospy.has_param('scale_x'):
            scale_x = rospy.get_param('scale_x')
            x = x*scale_x
        else:
            raise ValueError("scale_x not found")
        if rospy.has_param('scale_y'):
            scale_y = rospy.get_param('scale_y')
            y = y*scale_y
        else:
            raise ValueError("scale_y not found")
        if rospy.has_param('offset_x'):
            offset_x = rospy.get_param('offset_x')
            x -= offset_x
        else:
            raise ValueError("offset_x not found")
        if rospy.has_param('offset_y'):
            offset_y = rospy.get_param('offset_y')
            y -= offset_y
        else:
            raise ValueError("offset_y not found")

        # DuckPose (not published)
        pose = DuckPose()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "watchtower00/localization"
        if localized:
            pose.x = x
            pose.y = y
            pose.theta = theta
            pose.success = True
        else:
            pose.x = -1
            pose.y = -1
            pose.theta = -1
            pose.success = False


        # Odometry:
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.coordinates_pub.publish(odom)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = ImageFeature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down localization")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)