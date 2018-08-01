#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from std_msgs.msg import Int32




'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
Nalini 7/29/2018
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):

    	
        rospy.init_node('waypoint_updater')
        rospy.loginfo('in waypoint updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.loginfo('1. After current_pose')

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.loginfo('2. After base_waypoints')

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # added 7/1/2018
        # get pose of the car
        self.pose = msg
        # rospy.loginfo('4. After pose_cb')

        

    def waypoints_cb(self, msg):
        # TODO: Implement
        
        if self.waypoints is None:
        	self.waypoints = msg.waypoints
        
        rospy.loginfo('3. After waypoints_cb')


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # Nalini 7/25 - add later
        pass

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
    	dl = lambda a, b: math.sqrt((a.x-b.x)**2 - (a.y-b.y)**2)
    	for index, waypoint in enumerate(self.waypoints):
    		dist = dl(pose.position, waypoint.pose.pose.position)
    		if (dist < closest_len):
    			closest_len = dist
    			closest_waypoint = index

    	return closest_waypoint		



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
