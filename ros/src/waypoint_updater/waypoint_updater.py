#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from std_msgs.msg import Int32
import copy




'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
Nalini 11/25/2018
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5


class WaypointUpdater(object):
	def __init__(self):

		
		rospy.init_node('waypoint_updater')
		# rospy.loginfo('in waypoint updater')

		self.base_lane = None
		self.pose = None
		self.stopline_wp_idx = -1



		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
		# rospy.loginfo('1. After current_pose')

		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
		
		# rospy.loginfo('2. After base_waypoints')

		# TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)

		# rospy.loginfo('After traffic waypoint subscriber')


		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


		# TODO: Add other member variables you need below

		self.waypoints = None
		self.current_pose = None
		

		rospy.spin()


	def pose_cb(self, msg):
		# TODO: Implement
		# added 7/1/2018
		# get pose of the car
		self.current_pose = msg.pose
		# rospy.loginfo('4a. in Pose cb')
		if self.waypoints is not None:
			self.create_final_waypoints()

		# rospy.loginfo('4. After pose_cb')
		# rospy.loginfo(self.current_pose)

		

	def waypoints_cb(self, msg):
		# TODO: Implement
		
		if self.waypoints is None:
			self.waypoints = msg.waypoints
			self.base_lane = msg.waypoints
			
		# rospy.loginfo('3. After waypoints_cb')
		# rospy.loginfo(self.base_lane)


	def traffic_cb(self, msg):
		# TODO: Callback for /traffic_waypoint message. Implement
		# rospy.loginfo("in traffic_cb waypoint updater:")
		
		self.stopline_wp_idx = msg.data
		# rospy.loginfo(msg.data)
		

	def obstacle_cb(self, msg):
		# TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

	def get_closest_waypoint (self, pose):

		closest_len = 100000
		closest_waypoint = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
		
		for index, waypoint in enumerate(self.waypoints):
			dist = dl(pose.position, waypoint.pose.pose.position)
			if (dist < closest_len):
				closest_len = dist
				closest_waypoint = index

		return closest_waypoint     


	def publish_waypoints(self):
		final_lane = self.generate_lane()
		self.final_waypoints_pub.publish(final_lane)


	def genarate_lane(self):
		# get waypoints and update their velocities based on how we want the car to behave
		# if stop line waypoint is too far or -1, then just publish the waypoints, else decelerate the waypoints
		lane = Lane()

		closest_idx = self.get_closest_waypoint(self.current_pose)
		farthest_idx = closest_idx + LOOKAHEAD_WPS
		base_waypoints =  self.base_lane.waypoints[closest_idx:farthest_idx]

		if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
			lane.waypoints = base_waypoints
		else:
			lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

		return lane

	def decelerate_waypoints(self, waypoints, closest_idx):
		temp = []
		for i, wp in enumerate(waypoints):
			p = Waypoint()
			p.pose = wp.pose
			# stop two waypoints back from the line so that the car stops on time
			# -2 is included because the waypoint is at the center of teh car to we want to go two waypoints behind
			stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
			dist = self.distance(waypoints, i, stop_idx)
			# the larger the distance, larger the velocity
			vel = math.sqrt(2 * MAX_DECEL * dist)
			if vel < 1.:
				vel = 0.

			# if the calculated velocity is very large , then just get the waypoints velocity
			p.twist.twist.linear.x = min (vel, wp.twist.twist.linear.x)
			temp.append(p)				

		return temp

	def create_final_waypoints(self):
		# rospy.loginfo('in create final waypoints')
		if self.current_pose is not None:
			idx_of_nearest_wp = self.get_closest_waypoint (self.current_pose)
			# rospy.loginfo('nearest waypoint:')
			# rospy.loginfo(idx_of_nearest_wp)
			next_waypoints = copy.deepcopy(self.waypoints[idx_of_nearest_wp:idx_of_nearest_wp+LOOKAHEAD_WPS])
			lane = Lane()
			lane.header.frame_id = '/world'
			lane.waypoints = next_waypoints
			# rospy.loginfo(next_waypoints)
			self.final_waypoints_pub.publish(lane)
			# rospy.loginfo('Getting final waypoints')

	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
			wp1 = i
		return dist






if __name__ == '__main__':
	try:
		
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
