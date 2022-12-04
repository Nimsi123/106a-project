import rospy
import numpy as np
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

def actuator(trajectory):
  status_pub = rospy.Publisher('actuation_status', String, queue_size=10)
  
  # actuate the path

  status_pub.publish(f"Actuation completed. {trajectory}")

if __name__ == '__main__':
    rospy.init_node('sawyer_actuator', anonymous=True)

    rospy.Subscriber("actuation_path", String, actuator) # replace String with MultiDOFJointTrajectory
    rospy.spin()
