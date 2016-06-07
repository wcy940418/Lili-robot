#!/usr/bin/env python

# Error calculation node
# Listens for global_planner plan and amcl position


import numpy as np
import rospy as rp

from scipy import interpolate
from scipy.optimize import minimize
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
from move_base_msgs.msg import MoveBaseActionResult

def plan_callback(msg, arr):
    """Records the plans created by global_planner in a list

    Parameters:
    msg {Path} The heard message
    arr {list} List to store plans
    """
    x = lambda pose_stamped: pose_stamped.pose.position.x
    y = lambda pose_stamped: pose_stamped.pose.position.y
    pos_data = [[x(pose_stamped), y(pose_stamped)] for pose_stamped in msg.poses]
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

def result_callback(msg, (plans, actual, uncert)):
    """
    """
    success = msg.status.status == 3
    if success:
        path_planned = np.array(plans[-1]) # get most recent plan
        path_actual = np.array(actual)
        path_uncert = np.array(uncert)
        rp.loginfo(error_calc(path_planned, path_actual, path_uncert))

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
    f = interpolate.interp1d(xs, ys, fill_value="extrapolate")

    sse = 0.0
    for x_0, y_0 in actual:
        sq_dist = lambda x: (x_0 - x) ** 2 + (y_0 - f(x)) ** 2
        result = minimize(sq_dist, f(x_0))
        if result.success:
            sse += sq_dist(result.x[0]) # x should be 1x1 ndarray
        else:
            rp.logerr("Minimization failed; reason: %s" % result.message)

    n_points = actual.shape[0]
    return sse / n_points


def main():
    rp.init_node("listener", anonymous=True)

    paths_planned = []
    path_actual = []
    path_uncert = []

    rp.Subscriber("/move_base/DWAPlannerROS/local_plan",
                  data_class=Path,
                  callback=plan_callback,
                  callback_args=paths_planned)
    rp.Subscriber("/amcl_pose",
                  data_class=PWCS,
                  callback=pose_callback,
                  callback_args=(path_actual, path_uncert))
    rp.Subscriber("/move_base/result",
                  data_class=MoveBaseActionResult,
                  callback=result_callback,
                  callback_args=(paths_planned, path_actual, path_uncert))

    rp.spin()

if __name__ == "__main__":
    main()
