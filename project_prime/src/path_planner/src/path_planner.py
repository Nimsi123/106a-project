import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

def plan_path(p):
  path_pub = rospy.Publisher('actuation_path', String, queue_size=10) # replace String with MultiDOFJointTrajectory

  # plan the path
  path = f"Plan to point ({p.x}, {p.y}, {p.z})."

  path_pub.publish(path)

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)

  rospy.Subscriber("next_sawyer_loc", Point, plan_path)
  rospy.spin()
