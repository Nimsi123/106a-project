import rospy
import numpy as np
# import tf2_ros
from geometry_msgs.msg import Point

def test_main(max_publishing_freq):

  point_pub = rospy.Publisher('next_sawyer_loc', Point, queue_size=10)

  # Sets the minimum publishing rate by sleeping for 10hz
  sleeper = rospy.Rate(max_publishing_freq)

  # origin = np.array((0.4, 0, 0.3))
  origin = np.array((0.5, 0.5, 0.5))

  # flush the pipeline
  points = [
    np.array((0, 0, 0)), 
    np.array((0, 0, 0)), 
    np.array((0, 0, 0)),
    ]
  points = [origin + point for point in points]

  print("Flushing the pipeline.")
  for point in points:
    p = Point()
    p.x, p.y, p.z = point

    print(f"publishing {p}")
    point_pub.publish(p)
    sleeper.sleep()
  print("Pipeline flushed.")

  side_step = [
    np.array((0.53, 0.20, 0.08)), 
    np.array((0.70, 0.27, 0.12))
    ]

  box = [
    np.array((0.62, 0.7, 0.16)),
    np.array((0.67, 0.13, 0.91)),
    np.array((0.83, 0.08, 0.94)),
  ]

  box = [point / 4 for point in box]

  # points = side_step
  points = box
  points = [origin + point for point in points]

  for point in points:
    if rospy.is_shutdown():
      break

    p = Point()
    p.x, p.y, p.z = point

    print(f"publishing {p}")
    point_pub.publish(p)
    sleeper.sleep()

if __name__ == '__main__':
  rospy.init_node('hand_to_saywer_loc', anonymous=True)
  min_publishing_period = 3
  max_publishing_freq = 1 / min_publishing_period

  TEST = True
  if TEST:
    test_main(max_publishing_freq)
  else:
    main(max_publishing_freq)

