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
    arr {np.array} Array to store pose data
    """

def pose_callback(msg, arr, uncert):
    """Records the poses given by amcl in a np.array

    Parameters:
    msg {PWCS} The heard message
    arr {np.array} Array to store pose data
    uncert {np.array} Array to store uncertainty data
    """
    # TODO: Deal with uncertainty data

def main():
    rp.init_node("listener", anonymous=True)

    shape = [1,3] # TODO: quaternions?
    path_planned = np.array(shape)
    path_actual = np.array(shape)
    path_uncert = np.array(shape)

    rp.Subscriber("/global_planner/plan",
                  data_class=Path,
                  callback=plan_callback,
                  callback_args=path_planned)
    rp.Subscriber("/amcl/amcl_pose",
                  data_class=PWCS,
                  callback=pose_callback,
                  callback_args=(path_actual, path_uncert))

    rp.spin()

    # what to do with data after node is stopped?

if __name__ == "__main__":
    main()
