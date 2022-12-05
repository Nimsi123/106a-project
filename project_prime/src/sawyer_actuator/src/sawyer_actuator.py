import rospy
import numpy as np
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

def actuator(min_publishing_period):
  def actuator_helper(trajectory):
    status_pub = rospy.Publisher('actuation_status', String, queue_size=10)
    
    # actuate the path


    r = rospy.Rate(min_publishing_period)
    r.sleep()
    status_pub.publish(f"Actuation completed. {trajectory}")
  return actuator_helper

if __name__ == '__main__':
    rospy.init_node('sawyer_actuator', anonymous=True)
    min_publishing_period = 3
    
    rospy.Subscriber("actuation_path", String, actuator(min_publishing_period)) # replace String with MultiDOFJointTrajectory
    rospy.spin()
