import rospy
import numpy as np
from geometry_msgs.msg import Point
from trajectory_msgs.msg import MultiDOFJointTrajectory

def plan_path(p):
  path_pub = rospy.Publisher('actuation_path', MultiDOFJointTrajectory, queue_size=10)
  
  # plan the path

  path_pub.publish(plan)

if __name__ == '__main__':
    rospy.init_node('path_planner', anonymous=True)

    rospy.Subscriber("next_sawyer_loc", Point, plan_path)
    rospy.spin()
