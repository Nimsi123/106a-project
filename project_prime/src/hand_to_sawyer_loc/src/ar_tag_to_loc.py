import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers

class Buffer:
	def __init__(self, subscriber_topic, publisher_topic, output_frequency, process_input):
		self.buffer_store = []
		self.sleeper = rospy.Rate(output_frequency)
		self.process_input = process_input
		self.current_point = None

		self.pub = rospy.Publisher(publisher_topic, Point, queue_size=10)
		rospy.Subscriber(subscriber_topic, AlvarMarkers, self.read_in)

	def read_in(self, item):
		"""
		Reads an item from the input topic and processes it into the format 
		the output topic expects. Only adds to self.buffer_store if item is 'valid'.
		"""
		processed_item = self.process_input(item, self.current_point)
		if processed_item == None:
			return

		self.buffer_store.append(processed_item)

	def spin(self):
		"""
		Publishes to the output topic at a maximum frequency of self.output_frequency.
		"""
		while not rospy.is_shutdown():
			self.sleeper.sleep()
			if len(self.buffer_store) == 0:
				continue

			self.pub.publish(self.buffer_store[-1])
			self.current_point = self.buffer_store[-1]
			
			self.buffer_store = []
			

def pose_to_point(pose_item, current_point):
	if len(pose_item.markers) == 0:
		return None
	point = pose_item.markers[0].pose.pose.position

	p = Point()
	p.x = point.z
	p.y = -point.x
	p.z = -point.y

	# set the origin of the ar tag

	p.x -= 1
	p.z += 0.3

	# TUCK POSITION
	# origin = np.array((0.69, 0.16, 0.38))
	# set the origin to be the tuck position
	# when the AR tag is just in front of the camera
	# AR tag needs to be facing up.

	tuck_origin_x = 0.69
	tuck_origin_y = 0.16
	tuck_origin_z = 0.38

	# p.x += tuck_origin_x
	# p.y += tuck_origin_y
	# p.z += tuck_origin_z
	
	if current_point != None and \
		all(np.isclose(a, b, atol = 0.08) for a, b in [(current_point.x, p.x), (current_point.y, p.y), (current_point.z, p.z)]):
		return None

	return p


if __name__ == '__main__':
	rospy.init_node('camera_buffer', anonymous=True)
	min_publishing_period = 2
	max_publishing_freq = 1 / min_publishing_period

	b = Buffer("/ar_pose_marker", "/hand_loc", max_publishing_freq, pose_to_point)
	b.spin()