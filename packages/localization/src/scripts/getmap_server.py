#!/usr/bin/env python

import numpy as np
import cv2

from localization.srv import GetMap
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import CompressedImage
from scipy.interpolate import UnivariateSpline
# from localization.msg import Floats

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

def get_interpolation(img, no_preprocessing=False, return_origin=False, scaled=False, method="distance"):
    """
    Get the interpolation function of the trajectory of the agent in the environment.

    :param no_preprocessing: if True, the trajectory is not preprocessed
    :param return_origin: if True, the origin is returned
    :param scaled: if True, the coordinates are scaled
    :param method: if "angle", the angles are used, if "distance", the distance from starting point is used

    :return: np.array
    """
    top_view = img

    img_hsv = cv2.cvtColor(top_view, cv2.COLOR_RGB2HSV)
    gray = cv2.cvtColor(top_view, cv2.COLOR_RGB2GRAY)

    lower_yellow = np.array([20,100,150])
    upper_yellow = np.array([30,255,255])

    mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    mask = cv2.bitwise_and(gray, mask_yellow)

    if not no_preprocessing:
        kernel = np.ones((4, 4), np.uint8)
        eroded = cv2.erode(mask, kernel) 

        low_threshold = 89
        high_threshold = 80
        edges = cv2.Canny(eroded, low_threshold, high_threshold)

        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 3  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 10  # minimum number of pixels making up a line
        max_line_gap = 50  # maximum gap in pixels between connectable line segments

        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        _points = lines.reshape(-1, 2)

        x, y = _points.T

    else:
        x_all, y_all = np.nonzero(mask)
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

    s = 0.006 if scaled else 0.01

    spline_x = UnivariateSpline(spline_input, x_sorted, k=2, s=s)
    spline_y = UnivariateSpline(spline_input, y_sorted, k=2, s=s)

    if return_origin:
        return spline_x, spline_y, x_sorted, y_sorted, x0, y0

    return [spline_x, spline_y]


def handle_get_map_server(req):
    ros_data = rospy.wait_for_message("/watchtower00/camera_node/image/compressed", CompressedImage)
    np_arr = np.frombuffer(ros_data.data, 'u1')
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    res = get_interpolation(img, no_preprocessing=False, method="distance")
    return GetMap(res)

def get_map_server():
    rospy.init_node('get_map_server')
    s = rospy.Service('get_map', GetMap, handle_get_map_server)
    print("Ready to send a map.")
    rospy.spin()

if __name__ == "__main__":
    get_map_server()