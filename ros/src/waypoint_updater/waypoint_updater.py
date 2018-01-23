#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf
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
	self.wp_idx = None
        #rospy.spin()
	self.loop()
		
    def loop(self):
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():	
		if self.waypoints and self.wp_idx:
			next_waypoints = list(islice(cycle(self.waypoints),self.wp_idx,self.wp_idx+LOOKAHEAD_WPS-1))

			rospy.loginfo("closest_idx: %s"%self.wp_idx)
		    	waypoints2publish = Lane()
			waypoints2publish.header.frame_id = '/world'
			waypoints2publish.header.stamp = rospy.Time(0)
		   	waypoints2publish.waypoints = next_waypoints
			rospy.loginfo("publishing next waypoints!!")
		    	self.final_waypoints_pub.publish(waypoints2publish)
		rate.sleep()
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

    def find_nearest_waypoint(self):
	closest_dist = 1e6
	closest_idx = 0

	car_x = self.pose.position.x
	car_y = self.pose.position.y

	start_idx = 0
	stop_idx = len(self.waypoints)
	
	if self.wp_idx: 
		start_idx = max(self.wp_idx-50,0)
		stop_idx = min(self.wp_idx+50, len(self.waypoints))

	for idx in range(start_idx, stop_idx):
		wp = self.waypoints[idx]
		dist2car = math.sqrt((car_x-wp.pose.pose.position.x)**2 + (car_y-wp.pose.pose.position.y)**2)

	    	if dist2car<closest_dist:
			closest_dist = dist2car
			closest_idx = idx

	return closest_idx

    def calc_yaw(self):
	quaternion = (self.pose.orientation.x,
			self.pose.orientation.y,
			self.pose.orientation.z,
			self.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler[2]

    def calc_waypoints(self):

        if self.waypoints is None or self.pose is None:
            return

	closest_idx = self.find_nearest_waypoint()
	yaw = self.calc_yaw()

	wp = self.waypoints[closest_idx]

	car_x = self.pose.position.x
	car_y = self.pose.position.y

	wp_rel_car = (wp.pose.pose.position.x-car_x)*math.cos(yaw) + (wp.pose.pose.position.y-car_y)*math.sin(yaw)
	if wp_rel_car < 0.0:
		closest_idx +=1
	self.wp_idx = closest_idx
    

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

