#!/usr/bin/env python

import time

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage
from scipy.interpolate import UnivariateSpline
from localization.msg import Floats

from image_geometry import PinholeCameraModel

from dt_communication_utils import DTCommunicationGroup

group = DTCommunicationGroup('my_map', Floats)

LOW_RES=True

map_data = None

def sort_xy(x, y, return_origin=False):
    """
    Sort by angle

    :param return_origin: If true returns also the computed origin
    """

    x0 = np.mean(x)
    y0 = np.mean(y)

    r = np.sqrt((x-x0)**2 + (y-y0)**2)

    angles = np.where((y-y0) > 0, np.arccos((x-x0)/r), 2*np.pi-np.arccos((x-x0)/r))

    mask = np.argsort(angles)

    x_sorted = x[mask]
    y_sorted = y[mask]

    if return_origin:
        return x_sorted, y_sorted, x0, y0

    return x_sorted, y_sorted

def get_angles(x, y, x0=None, y0=None):
    """
    Get the angles of the trajectory.
    
    :param x: x coordinates
    :param y: y coordinates
    :param x0: x coordinate of the origin
    :param y0: y coordinate of the origin
    """

    if x0 is None:
        x0 = np.mean(x)
    if y0 is None:
        y0 = np.mean(y)

    r = np.sqrt((x-x0)**2 + (y-y0)**2)

    angles = np.where((y-y0) > 0, np.arccos((x-x0)/r), 2*np.pi-np.arccos((x-x0)/r))

    return angles

def get_interpolation(img, no_preprocessing=True, return_origin=False, scaled=False, method="distance"):
    """
    Get the interpolation function of the trajectory of the agent in the environment.

    :param no_preprocessing: if True, the trajectory is not preprocessed
    :param return_origin: if True, the origin is returned
    :param scaled: if True, the coordinates are scaled
    :param method: if "angle", the angles are used, if "distance", the distance from starting point is used

    :return: np.array
    """

    # https://github.com/duckietown/dt-core/blob/daffy/packages/complete_image_pipeline/include/complete_image_pipeline/calibrate_extrinsics.py
    # https://github.com/duckietown/dt-core/blob/daffy/packages/complete_image_pipeline/include/image_processing/rectification.py

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if LOW_RES:
        hsv_color1_yellow = np.array([30,35,187])
        hsv_color2_yellow = np.array([46,90,255])
    else:
        hsv_color1_yellow = np.array([30,70,170])
        hsv_color2_yellow = np.array([40,250,250])
        
    mask_yellow = cv2.inRange(img_hsv, hsv_color1_yellow, hsv_color2_yellow)

    x_all, y_all = np.nonzero(mask_yellow)
    x, y = x_all, y_all

    x_sorted, y_sorted, x0, y0 = sort_xy(x, y, return_origin=True)


    if method == "angle":
        # Interpolation angle-based
        angles = get_angles(x_sorted, y_sorted, x0=x0, y0=y0)
        # Add first and last point
        spline_input = np.concatenate([[0], angles, [2*np.pi]])
        x_sorted = np.concatenate([[x_sorted[-1]], x_sorted, [x_sorted[0]]])
        y_sorted = np.concatenate([[y_sorted[-1]], y_sorted, [y_sorted[0]]])
    elif method == "distance":
        # Interpolation distance-based
        points = np.array([x_sorted, y_sorted]).T
        distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
        spline_input = np.insert(distance, 0, 0)/distance[-1]
    else:
        raise ValueError("Unknown method, must be 'angle' or 'distance'")

    s = 0.01

    spline_x = UnivariateSpline(spline_input, x_sorted, k=2, s=s)
    spline_y = UnivariateSpline(spline_input, y_sorted, k=2, s=s)

    splines = [spline_x, spline_y]

    samples = 300

    if method == "angle":
        alpha = np.linspace(0, 2*np.pi, samples)
    elif method == "distance":
        alpha = np.linspace(0, 1, samples)
    else:
        raise ValueError("Unknown method, must be 'angle' or 'distance'")

    points_fitted = np.vstack( [spl(alpha) for spl in splines] ).T

    return points_fitted


def resize_params(points_fitted):
    """
    Resize the parameters of the interpolation function.

    :param points_fitted:
    :return:
    """

    # MEMO for dumb kids: x is row [1] and y is column [0]
    # this can be improved, should not be min and max but distance along same axis

    max_left = np.min(points_fitted[:, 0])
    max_right = np.max(points_fitted[:, 0])
    max_bottom = np.min(points_fitted[:, 1])
    max_top = np.max(points_fitted[:, 1])

    print("MAX Y: {}".format(max_top))
    print("MAX X: {}".format(max_right))
    print("MIN Y: {}".format(max_bottom))
    print("MIN X: {}".format(max_left))

    long_side, short_side = 2.36, 1.77
    env_long_len, env_short_len = 2.925, 2.34
    env_long_border, env_short_border = (env_long_len-long_side)/2, (env_short_len-short_side)/2

    scale_x = short_side / (max_top - max_bottom)
    scale_y = long_side / (max_right - max_left)

    offset_x = max_bottom*scale_y - env_short_border
    offset_y = max_left*scale_x - env_long_border

    print("Scale X: {}".format(scale_x))
    print("Scale Y: {}".format(scale_y))
    print("Offset X: {}".format(offset_x))
    print("Offset Y: {}".format(offset_y))

    rospy.set_param('scale_y', float(scale_y))
    rospy.set_param('scale_x', float(scale_x))
    rospy.set_param('offset_y', float(offset_y))
    rospy.set_param('offset_x', float(offset_x))

    points_fitted_resized = points_fitted * np.array([scale_x, scale_y])
    # points_fitted_resized[:, 0] -= offset_x
    # points_fitted_resized[:, 1] -= offset_y

    return points_fitted_resized

def get_rectification_params(msg, rectify_alpha=0.0):
    """
    Callback for the camera_info topic, first step to rectification.

    :param msg: camera_info message
    """
    # create mapx and mapy
    H, W = msg.height, msg.width
    # create new camera info
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(msg)
    # find optimal rectified pinhole camera
    rect_K, _ = cv2.getOptimalNewCameraMatrix(
        camera_model.K, camera_model.D, (W, H), rectify_alpha
    )
    # create rectification map
    _mapx, _mapy = cv2.initUndistortRectifyMap(
        camera_model.K, camera_model.D, None, rect_K, (W, H), cv2.CV_32FC1
    )
    return _mapx, _mapy

def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2RGB)
    return result

def process_map():
    msg = rospy.wait_for_message("/watchtower00/camera_node/camera_info", CameraInfo)
    print("[GetMap]: Got camera info")
    _mapx, _mapy = get_rectification_params(msg)
    ros_data = rospy.wait_for_message("/watchtower00/camera_node/image/compressed", CompressedImage)
    np_arr = np.frombuffer(ros_data.data, 'u1')
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # Rectify image
    image_rect = cv2.remap(img, _mapx, _mapy, cv2.INTER_NEAREST)
    print("[GetMap]: Remapped")
    img_wb = white_balance(image_rect)
    print("[GetMap]: White balanced")
    W, H = img_wb.shape[1], img_wb.shape[0]
    img_cutted = img_wb[int(H*0.15):int(H*0.78), int(W*0.2):int(W*0.8)]
    print("[GetMap]: Cutted")
    res = get_interpolation(img_cutted, no_preprocessing=True, method="distance")
    print(list(res.reshape(-1)))
    print("[GetMap]: Interpolation done")
    res_resized = resize_params(res)
    print("[GetMap]: Resized")
    res_flipped = res_resized
    # res_flipped[:,0] = res_resized[:,0].max() - res_resized[:,0]
    # print("[GetMap]: Flipped")
    return res_flipped.reshape(-1).astype(float).tolist()

def get_map_server():
    """
    Compute the map only the first time, than publish it using mudp.
    """
    if not map_data:
        rospy.init_node("send_map")
        print("[GetMap]: Starting map server")
        if LOW_RES:
            rospy.set_param("/watchtower00/camera_node/res_h", 324)
            rospy.set_param("/watchtower00/camera_node/res_w", 432)
            while rospy.get_param("/watchtower00/camera_node/res_h") != 324:
                rospy.sleep(0.1)
        print("[GetMap]: Low resolution set")
        map = process_map()
        print(map)
        print("[GetMap]: Map computed")
    else:
        map = map_data
        print("[GetMap]: Using stored map")
    publisher = group.Publisher()
    msg = Floats(map)
    while not rospy.is_shutdown():
        publisher.publish(msg)
        rospy.sleep(2)

if __name__ == "__main__":
    get_map_server()