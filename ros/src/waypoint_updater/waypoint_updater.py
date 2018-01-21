#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from itertools import cycle, islice

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	#rospy.Subscriber('/traffic_waypoint', ,self.traffic_cb)
	#rospy.Subscriber('/obstacle_waypoint', ,self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.pose = None
	self.waypoints = None
	self.prev_wp_idx = None
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.pose = msg.pose
	self.calc_waypoints()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	self.waypoints = waypoints.waypoints
	self.calc_waypoints()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def calc_waypoints(self):
        if self.waypoints is None or self.pose is None:
            return

	car_x = self.pose.position.x
	car_y = self.pose.position.y
	
	closest_dist = -1
	closest_idx = -1
	for idx in range(len(self.waypoints)):
	    wp = self.waypoints[idx]
	    dist2car = math.sqrt((car_x-wp.pose.pose.position.x)**2 + (car_y-wp.pose.pose.position.y)**2)
	    if closest_dist < 0:
		closest_dist = dist2car
		closest_idx = idx
	    elif dist2car<closest_dist:
		closest_dist = dist2car
		closest_idx = idx

	next_waypoints = list(islice(cycle(self.waypoints),closest_idx,closest_idx+LOOKAHEAD_WPS-1))

    	waypoints2publish = Lane()
   	waypoints2publish.waypoints = next_waypoints
	rospy.loginfo("publishing next waypoints!!")
    	self.final_waypoints_pub.publish = waypoints2publish
    

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
