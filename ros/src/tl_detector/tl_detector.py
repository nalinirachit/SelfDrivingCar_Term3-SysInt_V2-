#!/usr/bin/env python




import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math


'''
Nalini 1/6/2019
appears to return 0 but still does not work
int: ID of traffic light color (specified in styx_msgs/TrafficLight)

uint8 UNKNOWN=4
uint8 GREEN=2
uint8 YELLOW=1
uint8 RED=0
/home/student/CarND-Capstone-master/ros/src/styx_msgs/msg

'''

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
	def __init__(self):
		rospy.init_node('tl_detector')

		self.pose = None
		self.waypoints = None
		self.camera_image = None
		self.lights = []

		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		'''
		/vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
		helps you acquire an accurate ground truth data source for the traffic light
		classifier by sending the current color state of all traffic lights in the
		simulator. When testing on the vehicle, the color state will not be available. You'll need to
		rely on the position of the light and the camera image to predict it.
		'''
		sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

		sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

		config_string = rospy.get_param("/traffic_light_config")
		self.config = yaml.load(config_string)

		self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

		self.bridge = CvBridge()
		self.light_classifier = TLClassifier()
		self.listener = tf.TransformListener()

		self.state = TrafficLight.UNKNOWN
		self.last_state = TrafficLight.UNKNOWN
		self.last_wp = -1
		self.state_count = 0

		rospy.spin()

	def pose_cb(self, msg):
		self.pose = msg

	def waypoints_cb(self, waypoints):
		self.waypoints = waypoints.waypoints

	def traffic_cb(self, msg):
		# rospy.loginfo("In traffic_cb")
		self.lights = msg.lights

	def distance(self, x1, y1, x2, y2):
		dx, dy = x1-x2, y1-y2
		dist = math.sqrt(dx*dx + dy*dy)
		return dist

	def image_cb(self, msg):
		"""Identifies red lights in the incoming camera image and publishes the index
			of the waypoint closest to the red light's stop line to /traffic_waypoint

		Args:
			msg (Image): image from car-mounted camera

		"""
		self.has_image = True
		self.camera_image = msg
		# rospy.loginfo("In image_cb")
		light_wp, state = self.process_traffic_lights()
		# rospy.loginfo("got light state")
		# rospy.loginfo("light_wp and state", light_wp, state)

		'''
		Publish upcoming red lights at camera frequency.
		Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
		of times till we start using it. Otherwise the previous stable state is
		used.
		'''
		if self.state != state:
			self.state_count = 0
			self.state = state
		elif self.state_count >= STATE_COUNT_THRESHOLD:
			self.last_state = self.state
			light_wp = light_wp if state == TrafficLight.RED else -1
			self.last_wp = light_wp
			self.upcoming_red_light_pub.publish(Int32(light_wp))
		else:
			self.upcoming_red_light_pub.publish(Int32(self.last_wp))
		self.state_count += 1

	def get_closest_waypoint(self, x, y):
		"""Identifies the closest path waypoint to the given position
			https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
		Args:
			pose (Pose): position to match a waypoint to

		Returns:
			int: index of the closest waypoint in self.waypoints

		"""
		#TODO implement

		# rospy.loginfo("In get Closest WP:")
		# rospy.loginfo(x)
		# rospy.loginfo(y)

		closest_len = 100000
		closest_waypint = 0

		# not working - creating separate function
		# dl = lambda x, y , a, b: math.sqrt ((a.x-b.x)**2 + (a.y-b.y)**2)


		for index, waypoint in enumerate(self.waypoints):
			dist = self.distance(x, y, waypoint.pose.pose.position.x, waypoint.pose.pose.position.x)
			if dist < closest_len:
				closest_len = dist
				closest_waypoint =  index

		return closest_waypoint





	def get_light_state(self, light):
		"""Determines the current color of the traffic light

		Args:
			light (TrafficLight): light to classify

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""

		# for testing just return light stats
		rospy.loginfo("Light State")
		rospy.loginfo(light.state)
		return light.state

		# 9/8/2018 comment out for now
		# if(not self.has_image):
			# self.prev_light_loc = None
			# return False

		# 9/8/2018 comment out for now
		# cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

		#Get classification
		# 9/8/2018 comment out for now
		# return self.light_classifier.get_classification(cv_image)

	def process_traffic_lights(self):
		"""Finds closest visible traffic light, if one exists, and determines its
			location and color

		Returns:
			int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		closest_light = None
		line_wp_idx = None


		# List of positions that correspond to the line to stop in front of for a given intersection
		stop_line_positions = self.config['stop_line_positions']

		if(self.pose):
			# rospy.loginfo("car position:")
			# rospy.loginfo(self.pose)
			car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y )
			diff = len(self.waypoints)

			for i, light in enumerate(self.lights):
				line = stop_line_positions[i]
				# rospy.loginfo("line values:")
				# rospy.loginfo(line)
				temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
				# find closest waypoint index
				d = temp_wp_idx - car_wp_idx
				if d > 0 and d < diff:
					diff = d
					closest_light = light
					line_wp_idx = temp_wp_idx

		#TODO find the closest visible traffic light (if one exists)
		if closest_light:
			state = self.get_light_state(closest_light)
			rospy.loginfo("tl_detector returning values:")
			rospy.loginfo(line_wp_idx)
			rospy.loginfo(state)
			return line_wp_idx, state       
		
		return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
	try:
		TLDetector()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start traffic node.')
