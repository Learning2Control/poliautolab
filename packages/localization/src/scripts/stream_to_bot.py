#!/usr/bin/env python

"""Detects duckiebots from the watchtower using their unique colors.

Publish on watchtower00/localization a DuckPose with the car coordinates.
Publish using dt_communication.
"""
__author__ =  'Giulio Vaccari <giulio.vaccari at mail.polimi.it>'
__version__=  '0.1'
__license__ = 'MIT'

# Python libs
import sys, os, copy

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import rospy
import tf
from image_geometry import PinholeCameraModel

# Ros Messages
from sensor_msgs.msg import CameraInfo, CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import CameraInfo, CompressedImage
from localization.msg import DuckPose

# Duckie stuff
from dt_communication_utils import DTCommunicationGroup
from duckietown.dtros import DTROS, NodeType

# UDP communication
group = DTCommunicationGroup('my_position', DuckPose)

#### Params ####
VERBOSE=False
PUB_RECT=False
PUB_ROS=False

#### Resolution params ####
LOW_RES=False
SUPER_LOW_RES=True
assert LOW_RES != SUPER_LOW_RES
FIX_SCALE = 4 if SUPER_LOW_RES else 3 if LOW_RES else 1
if SUPER_LOW_RES:
    canvas_for_circle = np.zeros(shape=(153,195))

def get_car(img):
    """
    Extract the car from the image.

    :param img: image
    :return: front coord, left coord, theta
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Extract blue and pink colors
    if LOW_RES:

        hsv_color1_blue = np.array([87, 117, 170])
        hsv_color2_blue = np.array([116, 189, 255])
        mask_blue = cv2.inRange(img_hsv, hsv_color1_blue, hsv_color2_blue)

        hsv_color1_pink = np.array([130, 40, 190])
        hsv_color2_pink = np.array([255, 255, 255])
        mask_pink = cv2.inRange(img_hsv, hsv_color1_pink, hsv_color2_pink)

    elif SUPER_LOW_RES:

        hsv_color1_blue = np.array([50, 100, 150])
        hsv_color2_blue = np.array([110, 255, 255]) #[102, 255, 255] without the circle processing
        mask_blue = cv2.inRange(img_hsv, hsv_color1_blue, hsv_color2_blue)

        # Pink is the same!
        hsv_color1_pink = np.array([130, 40, 190])
        hsv_color2_pink = np.array([255, 255, 255])
        mask_pink = cv2.inRange(img_hsv, hsv_color1_pink, hsv_color2_pink)

    else:

        hsv_color1_blue = np.array([80, 180, 110])
        hsv_color2_blue = np.array([200, 250, 250])
        mask_blue = cv2.inRange(img_hsv, hsv_color1_blue, hsv_color2_blue)

        hsv_color1_pink = np.array([150, 50, 100])
        hsv_color2_pink = np.array([200, 100, 250])
        mask_pink = cv2.inRange(img_hsv, hsv_color1_pink, hsv_color2_pink)
    
    # Compute the average location the blue and pink colors
    back_coo = np.argwhere(mask_pink==255).mean(axis=0)[::-1]
    if np.isnan(back_coo[0]):
        raise ValueError('No pink color found')
    if SUPER_LOW_RES:
        # Bounding box to be able to use a wide range for blue
        mask_blue = cv2.bitwise_and(cv2.circle(np.zeros_like(mask_blue), (int(back_coo[0]), int(back_coo[1])), 15, (255), -1), mask_blue)
    front_coo = np.argwhere(mask_blue==255).mean(axis=0)[::-1]
    
    # Compute the position of the car as the average between the position of the back and front
    x_center = (front_coo[0] + back_coo[0])/2
    y_center = (front_coo[1] + back_coo[1])/2
    if np.isnan(x_center):
        print(front_coo[0], back_coo[0])

    # Compute the angle of the car
    angle = np.arctan2(-front_coo[0]+back_coo[0], -front_coo[1]+back_coo[1]) # x and y are inverted because of the image coordinate system
    
    return x_center, y_center, angle

def white_balance(img):
    """
    White balance the image using the gray world assumption.
    
    :param img: image

    :return: white balanced image
    """
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

class ImageFeature(DTROS):

    def __init__(self, node_name):
        """
        Initialize the ImageFeature class.
        """

        # Duck name
        self.vehicle = os.environ['VEHICLE_NAME']

        # Initialize the DTROS parent class
        # https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown/include/duckietown/dtros/constants.py
        super(ImageFeature, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # Camera info
        self.rectify_alpha = rospy.get_param("~rectify_alpha", 0.0)
        self._camera_parameters = None
        self._mapx, self._mapy = None, None

        # Scale factor
        rospy.loginfo('[Watcher]: Waiting for parameter server...')
        # while not rospy.has_param('scale_x'):
        #     self.rate.sleep()
        # Muktiple by FIX_SCALE to adapt to resolution
        self.scale_x = 0.00345041662607739*FIX_SCALE #rospy.get_param('scale_x', 0.00345041662607739)
        self.scale_y = 0.005417244522218992*FIX_SCALE #rospy.get_param('scale_y', 0.005417244522218992)
        rospy.loginfo('[Watcher]: Got params.')

        # Publishers
        self.coordinates_dt_publish = group.Publisher()
        if PUB_ROS:
            self.coordinates_pub = rospy.Publisher("/watchtower00/localization", Odometry, queue_size=1)
            self.odom_broadcaster = tf.TransformBroadcaster()
        if PUB_RECT:
            self.image_pub = rospy.Publisher("/watchtower00/image_rectified/compressed", CompressedImage, queue_size=1)

        # Subscribers
        # https://stackoverflow.com/questions/33559200/ros-image-subscriber-lag?rq=1
        self._cinfo_sub = rospy.Subscriber("/watchtower00/camera_node/camera_info",
            CameraInfo, self._cinfo_cb, queue_size=1)
        self.image_subscriber = rospy.Subscriber("/watchtower00/camera_node/image/compressed",
            CompressedImage, self._img_cb, queue_size=1, buff_size=2**24)

        if VERBOSE :
            print("ImageFeature initialized")

    def _cinfo_cb(self, msg):
        """
        Callback for the camera_info topic, first step to rectification.

        :param msg: camera_info message
        """
        if VERBOSE :
            print("subscribed to /camera_info")
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        # find optimal rectified pinhole camera
        rect_K, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_model.K, self.camera_model.D, (W, H), self.rectify_alpha
        )
        # # store new camera parameters
        # self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])
        # create rectification map
        self._mapx, self._mapy = cv2.initUndistortRectifyMap(
            self.camera_model.K, self.camera_model.D, None, rect_K, (W, H), cv2.CV_32FC1
        )
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def _img_cb(self, ros_data):
        """
        Callback function for subscribed topic.
        Get image and extract position and orientation of Duckiebot.
        Publish duckiebot position and orientation on /watchtower00/localization.

        :param ros_data: received image

        :type ros_data: sensor_msgs.msg.CompressedImage
        """
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        start_time = rospy.get_time()

        # To numpy array
        np_arr = np.frombuffer(ros_data.data, 'u1')
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Rectify
        remapped = cv2.remap(image_np, self._mapx, self._mapy, cv2.INTER_NEAREST)

        # White balance
        img = white_balance(remapped)

        # Cut image
        W, H = img.shape[1], img.shape[0]
        img = img[int(H*0.15):int(H*0.78), int(W*0.2):int(W*0.8)]
        # MEMO: Img has origin on top left, after the interpolation it will be rotated of 90 degrees, no need to rotate it back

        # Compute position and orientation
        localized = False
        try:
            x, y, theta = get_car(img)
            localized = True
        except ValueError:
            print("No lines found.")
            localized = False
            x, y, theta = -1, -1, -1
        
        if np.isnan(x) or np.isnan(y) or np.isnan(theta):
            print("No lines found.")
            localized = False

        x, y = y, x # Because of the different methods between map creation and localization x and y are flipped

        if VERBOSE:
            print("Pixel: x: ", x, "y: ", y)

        # Resize and remove offset
        x = x*self.scale_x
        y = y*self.scale_y

        # Transform to fit updated map
        leftmost_x = 0.36167528287773865
        short_coeff_x = 1.2333649447885058
        x_after_transformation = (x-leftmost_x)*short_coeff_x + leftmost_x

        bottom_y = 0.5875344705138967
        short_coeff_y = 0.8858931614382081
        y_after_transformation = (y-bottom_y)*short_coeff_y + bottom_y

        x, y = x_after_transformation, y_after_transformation

        # Remove offset
        # offset_x = rospy.get_param('offset_x', 0.3598180360213129)
        # x -= offset_x
        # offset_y = rospy.get_param('offset_y', 0.07411439522846053)
        # y -= offset_y
        # if not rospy.has_param('offset_x'):
        #     print("[STREAM_TO_BOT] params not found")

        # Prepare DuckPose message
        pose = DuckPose()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "watchtower00/localization"
        pose.x = x
        pose.y = y
        pose.theta = theta
        pose.success = localized

        # Publish duckiebot position and orientation as UDP packet
        print(f"[Watcher]: publishing x:{x}, y:{y}, theta:{np.rad2deg(theta)}")
        self.coordinates_dt_publish.publish(pose)

        # Publish the processed image for debugging
        if PUB_RECT:
            # NB It is correct wrt to the map, not the image thus we need to flip along y
            img_to_be_pulished = copy.deepcopy(img)
            # Create CompressedImage
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            img_to_be_pulished = cv2.rotate(img_to_be_pulished, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.circle(img_to_be_pulished,(int(x), int(img.shape[1]-y)), 25, (0,255,0))
            msg.data = np.array(cv2.imencode('.jpg', img_to_be_pulished)[1]).tobytes()
            # Publish image
            self.image_pub.publish(msg)

        # Publish odometry for debugging
        if PUB_ROS:
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

        print("Total time: ", rospy.get_time()-start_time)

def main():
    """
    Main function, initialize node and start the watcher.
    """
    print("[Watcher]: Starting...")

    # Set resolution level
    if LOW_RES:
        rospy.set_param("/watchtower00/camera_node/res_h", 324)
        rospy.set_param("/watchtower00/camera_node/res_w", 432)
        while rospy.get_param("/watchtower00/camera_node/res_h") != 324:
            rospy.sleep(0.1)
        print("[Watcher]: Low resolution set")
    elif SUPER_LOW_RES:
        rospy.set_param("/watchtower00/camera_node/res_h", 243)
        rospy.set_param("/watchtower00/camera_node/res_w", 324)
        while rospy.get_param("/watchtower00/camera_node/res_h") != 243:
            rospy.sleep(0.1)
        print("[Watcher]: Super low resolution set")
    
    ic = ImageFeature(node_name='watcher')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("[Watcher]: Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()