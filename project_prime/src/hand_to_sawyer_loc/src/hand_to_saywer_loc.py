import rospy
import numpy as np
# import tf2_ros
from geometry_msgs.msg import Point

def test_main(max_publishing_freq):

  point_pub = rospy.Publisher('next_sawyer_loc', Point, queue_size=10)

  # Sets the minimum publishing rate by sleeping for 10hz
  sleeper = rospy.Rate(max_publishing_freq)

  r = 0.8
  time_steps = np.linspace(-np.pi / 2, np.pi / 2, 5)
  x = r * np.cos(time_steps)
  y = r * np.sin(time_steps)
  z = 0.5 * np.ones(len(time_steps))
  points = np.vstack((x, y, z))

  for i in range(points.shape[1]):
    if rospy.is_shutdown():
      break

    p = Point()
    p.x, p.y, p.z = points[:, i]

    print("publishing")
    point_pub.publish(p)
    sleeper.sleep()

if __name__ == '__main__':
  rospy.init_node('hand_to_saywer_loc', anonymous=True)
  min_publishing_period = 3
  max_publishing_freq = min_publishing_period / 3

  TEST = True
  if TEST:
    test_main(max_publishing_freq)
  else:
    main(max_publishing_freq)

