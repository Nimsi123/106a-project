import rospy
import numpy as np
# import tf2_ros
from geometry_msgs.msg import Point

def test_main(min_publishing_period):

  point_pub = rospy.Publisher('next_sawyer_loc', Point, queue_size=10)
  # tfBuffer = tf2_ros.Buffer()
  # tfListener = tf2_ros.TransformListener(tfBuffer) # which is primed with a tf listener
  
  # Sets the minimum publishing rate by sleeping for 10hz
  r = rospy.Rate(min_publishing_period)

  counter = 0
  while not rospy.is_shutdown():
    # try:
      # trans = tfBuffer.lookup_transform(turtlebot_frame, goal_frame, rospy.Time())
      # ...
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #   pass

    p = Point()
    p.x = counter
    p.y = counter
    p.z = counter

    point_pub.publish(p)

    r.sleep()
    counter = (counter + 1) % 100

if __name__ == '__main__':
  rospy.init_node('hand_to_saywer_loc', anonymous=True)
  min_publishing_period = 3

  TEST = True
  if TEST:
    test_main(min_publishing_period)
  else:
    main(min_publishing_period)

