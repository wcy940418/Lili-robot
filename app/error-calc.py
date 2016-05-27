"""
Error calculation node
Listens for global_planner plan and amcl position
"""

import numpy as np
import rospy as rp

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS

def plan_callback(msg, arr):
    """Records most recent plan created by global_planner in a list

    Parameters:
    msg {Path} The heard message
    arr {list} List to store pose data
    """

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

def main():
    rp.init_node("listener", anonymous=True)

    shape = (1,3) # TODO: quaternions?
    path_planned = []
    path_actual = []
    path_uncert = []

    rp.Subscriber("/global_planner/plan",
                  data_class=Path,
                  callback=plan_callback,
                  callback_args=path_planned)
    rp.Subscriber("/amcl/amcl_pose",
                  data_class=PWCS,
                  callback=pose_callback,
                  callback_args=(path_actual, path_uncert))

    rp.spin()

    path_planned = np.array(path_planned)
    path_actual = np.array(path_actual)
    path_uncert = np.array(path_uncert)
