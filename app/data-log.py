#!/usr/bin/env python

# Error calculation node
# Listens for global_planner plan and amcl position


import numpy as np
import rospy as rp
import tf
import time
import os
import itertools
import rospkg
import csv

# from collections import namedtuple
from tf import transformations as trans
from scipy import interpolate
from scipy.optimize import minimize
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
from move_base_msgs.msg import MoveBaseActionResult

def plan_callback(msg, (l, arr)):
    """Records the plans created by global_planner in a list

    Parameters:
    msg {Path} The heard message
    l {TransformListener} The transform listener
    arr {list} List to store plans
    """
    # transform poses from local frame to global frame
    transformed_poses = [l.transformPose('/map', pose_stamped) for pose_stamped in msg.poses]
    
    x = lambda pose_stamped: pose_stamped.pose.position.x
    y = lambda pose_stamped: pose_stamped.pose.position.y
    pos_data = [[x(pose_stamped), y(pose_stamped)] for pose_stamped in transformed_poses]
    arr.append(pos_data)

def pose_callback(msg, (arr, uncert)):
    """Records the poses given by amcl in a list

    Parameters:
    msg {PWCS} The heard message
    arr {list} List to store pose data
    uncert {list} List to store uncertainty data
    """
    # TODO: Deal with uncertainty data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # z = msg.pose.position.z
    arr.append([x, y]) # assume on 2D plane

def result_callback(msg, (p, plans, actual, uncert)):
    """If goal is successfully reached, initalize data logging

    Parameters:
    p {str} The path to the data directory
    plans {list} A list of lists containing all the published local plans 
    actual {list} A list containing the pose estimates 
    uncert {list} A list containing the uncertainty in pose estimates
    """
    # rp.loginfo(str(plans))
    # rp.loginfo(str(actual))
    success = msg.status.status == 3
    if success:
        data_log(p, plans, actual, uncert)
        # path_planned = np.array(plans[-1]) # get most recent plan
        # path_actual = np.array(actual)
        # path_uncert = np.array(uncert)
        # rp.loginfo(error_calc(path_planned, path_actual, path_uncert))

def data_log(p, plans, actual, uncert):
    """Logs the data to .csv files
    """
    # TODO: Need to handle uncertainty
    data_types = ['plan', 'actual']
    field_names = ['x', 'y']

    for data_type in data_types:
        if data_type == 'plan':
            data = itertools.chain.from_iterable(plans)
        elif data_type == 'actual':
            data = actual

        filename = p + '/' + data_type + '.csv'
        is_new_file = False
        if not os.path.exists(filename):
            f = open(filename, 'w+b')
            is_new_file = True
        else:
            f = open(filename, 'a+b')

        with f:
            writer = csv.DictWriter(f, fieldnames=field_names)
            if is_new_file:
                writer.writeheader()
            for x, y in data:
                writer.writerow({'x': x, 'y': y})

'''
def error_calc(plan, actual, uncert):
    """Given a planned path `plan`, actual path `actual`, and uncertainty in
    path `uncert`, calculate the error.

    Parameters:
    plan {np.array} The planned path
    actual {np.array} The actual path
    uncert {np.array} The uncertainty in path

    Returns: {float64} The mean squared error
    """
    # TODO: how to handle uncertainty?
    xs = plan[:, 0]
    ys = plan[:, 1]

    # continuous and piecewise linear interpolation
    # extrapolate to handle values of x outside of the domain of f
    # TODO: handle extrapolation--only available in version 0.17
    f = interpolate.interp1d(xs, ys)

    sse = 0.0
    for x_0, y_0 in actual:
        sq_dist = lambda x: (x_0 - x) ** 2 + (y_0 - f(x)) ** 2
        try:
            result = minimize(sq_dist, f(x_0))
        except ValueError: # x_0 is out of bounds
            rp.logerr("x_0 out of bounds")
            continue
        if result.success:
            sse += sq_dist(result.x[0]) # x should be 1x1 ndarray
        else:
            rp.logerr("Minimization failed; reason: %s" % result.message)

    n_points = actual.shape[0]
    return sse / n_points
'''

def uncolonify(s):
    """Replace the colons in `s` with `-`
    """
    new_s = ''
    for char in s:
        if char == ':':
            new_s += '-'
        else:
            new_s += char

    return new_s

def init_data_directory(t):
    """Given a string `t` representing the current time, initialize a directory
    for data collection and return the path to that directory

    Returns: {str} The path to the newly created directory
    """
    rospack = rospkg.RosPack()
    lili_path = rospack.get_path('lili_navi')
    # lili_path = os.path.abspath('./src/lili_navi')
    # NB: assume that this file lives in a child directory of 'lili_navi'
    assert os.path.basename(lili_path) == 'lili_navi', lili_path

    data_path = lili_path + '/data'
    if not os.path.exists(data_path):
        os.mkdir(data_path)

    curr_path = data_path + '/%s' % t
    os.mkdir(curr_path)

    return curr_path

def main():
    start = time.asctime()
    start = '-'.join(start.split())
    start = uncolonify(start)

    data_path = init_data_directory(start)

    rp.init_node("listener", anonymous=True)

    paths_planned = []
    path_actual = []
    path_uncert = []
    
    l = tf.TransformListener()

    rp.Subscriber("/move_base/DWAPlannerROS/local_plan",
                  data_class=Path,
                  callback=plan_callback,
                  callback_args=(l, paths_planned))
    rp.Subscriber("/amcl_pose",
                  data_class=PWCS,
                  callback=pose_callback,
                  callback_args=(path_actual, path_uncert))
    rp.Subscriber("/move_base/result",
                  data_class=MoveBaseActionResult,
                  callback=result_callback,
                  callback_args=(data_path, paths_planned, path_actual, path_uncert))

    rp.spin()

if __name__ == "__main__":
    main()
