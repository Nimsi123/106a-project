#!/usr/bin/env python

import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
    
def create_goal(x, y, z):
    goal = PoseStamped()
    goal.header.frame_id = "base"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    return goal

def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    desired_points = [np.array((0, 0, 1)) + (t / 30) * np.array((0, 1, -1)) for t in range(30)]

    for x, y, z in desired_points:
        print('Position')
        print(x)
        print(y)
        print(z)
        if rospy.is_shutdown():
            break

        try:
            plan = planner.plan_to_pose(create_goal(x,y,z), [])
            if not planner.execute_plan(plan[1]): 
                raise Exception("Execution failed")
        except Exception as e:
            print(e)
            traceback.print_exc()


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
