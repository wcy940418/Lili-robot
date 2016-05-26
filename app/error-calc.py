"""
Error calculation node
Listens for global_planner plan and amcl position
"""

import numpy as np
import rospy as rp

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS

def plan_callback(msg, arr):
    """Records most recent plan created by global_planner in a np.array

    Parameters:
    msg {Path} The heard message
    arr {np.array} Array to store pose information
    """

def pose_callback(msg, arr):
    """Records the poses given by amcl in a np.array

    Parameters:
    msg {PWCS} The heard message
    arr {np.array} Array to store pose information
    """
    # TODO: Deal with covariance data

def main():
    rp.init_node("listener", anonymous=True)

    path_planned = np.array()
    path_actual = np.array()

    rp.Subscriber("/global_planner/plan", Path, plan_callback, path_planned)
    rp.Subscriber("/amcl/amcl_pose", PWCS, pose_callback, path_actual)
