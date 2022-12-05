import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

def plan_path(min_publishing_period):
  def plan_path_helper(p):
    path_pub = rospy.Publisher('actuation_path', String, queue_size=10) # replace String with MultiDOFJointTrajectory

    # plan the path
    path = f"Plan to point ({p.x}, {p.y}, {p.z})."


    r = rospy.Rate(min_publishing_period)
    r.sleep()
    path_pub.publish(path)
  return plan_path_helper

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)
  min_publishing_period = 3

  rospy.Subscriber("next_sawyer_loc", Point, plan_path(3))
  rospy.spin()
