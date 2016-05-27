"""
Error calculation node
Listens for global_planner plan and amcl position
"""

import numpy as np
import rospy as rp

from scipy import interpolate
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS

def plan_callback(msg, arr):
    """Records the plans created by global_planner in a list

    Parameters:
    msg {Path} The heard message
    arr {list} List to store plans
    """
    x = lambda pose: pose.position.x
    y = lambda pose: pose.position.y
    pos_data = [[x(pose), y(pose)] for pose in msg.poses]
    arr.append(pos_data)

def pose_callback(msg, arr, uncert):
    """Records the poses given by amcl in a list

    Parameters:
    msg {PWCS} The heard message
    arr {list} List to store pose data
    uncert {np.array} Array to store uncertainty data
    """
    # TODO: Deal with uncertainty data
    x = msg.pose.position.x
    y = msg.pose.position.y
    # z = msg.pose.position.z
    arr.append([x, y]) # assume on 2D plane

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
    f = interpolate.interp1d(xs, ys)

    sse = np.float64(0)
    for x, y in actual:
        sq_err = (f(x) - y) ** 2
        sse += sq_err

    return sse / actual.shape[1]


def main():
    rp.init_node("listener", anonymous=True)

    paths_planned = []
    path_actual = []
    path_uncert = []

    rp.Subscriber("/global_planner/plan",
                  data_class=Path,
                  callback=plan_callback,
                  callback_args=paths_planned)
    rp.Subscriber("/amcl/amcl_pose",
                  data_class=PWCS,
                  callback=pose_callback,
                  callback_args=(path_actual, path_uncert))

    rp.spin()

    path_planned = np.array(paths_planned[-1]) # get most recent plan
    path_actual = np.array(path_actual)
    path_uncert = np.array(path_uncert)
    error_calc(path_planned, path_actual, path_uncert)

if __name__ == "__main__":
    main()
