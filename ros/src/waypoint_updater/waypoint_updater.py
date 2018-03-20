#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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
	#rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb);

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

	# Current position of the vehicle
	self.x = 0
	self.y = 0
	self.theta = 0
	
	self.rate = rospy.Rate(10)
    	self.startTime = 0

        rospy.spin()

    def pose_cb(self, msg):
	pose = msg.pose
	self.x = pose.position.x
	self.y = pose.position.y
	# Convert Quaternion to Euler
	self.theta = pose.orientation.z	
		
	rospy.loginfo('Vehicle Pose Received - x:%s, y:%s, theta:%s', self.x, self.y, self.theta)

	while not self.startTime:
        	self.startTime = rospy.Time.now().to_sec()

    	while not rospy.is_shutdown():
        	pub_j2.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        	rate.sleep()

    def waypoints_cb(self, msg):
	waypoints = msg.waypoints	
	x = self.x
	y = self.y
	theta = self.theta
	# Find closest waypoint
	closestWaypoint, closestWaypointIdx = self.NextWaypoint(waypoints, x, y, theta)
	
	wpX = closestWaypoint.pose.pose.position.x
	wpY = closestWaypoint.pose.pose.position.y
	rospy.loginfo('Closest Waypoint: %s, x:%s, y:%s', closestWaypointIdx, wpX, wpY) 
	
	finalWaypoints = []

	for i in xrange(LOOKAHEAD_WPS):
		waypointIdx = (closestWaypointIdx + i) % len(waypoints)
		waypoint = waypoints[waypointIdx]
		finalWaypoints.append(waypoint)

	self.final_waypoints_pub.publish(finalWaypoints)

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

    def distanceWP(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def distance(self, x1, y1, x2, y2):
    	return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    def ClosestWaypoint(self, waypoints, x, y):
    	closestLen = 100000 # large number
    	closest2ndLen = 1000000
    	closestWaypointIdx = 0

    	for i, waypoint in enumerate(waypoints):
        	wpX = waypoint.pose.pose.position.x
        	wpY = waypoint.pose.pose.position.y
        	dist = self.distance(x, y, wpX, wpY)
        	if (dist < closestLen):
            		closestLen = dist
            		closestWaypointIdx = i
			closestWaypoint = waypoint
        	if (dist < closest2ndLen and closest2ndLen > closestLen):
            		closest2ndLen = dist
            		closest2ndWaypointIdx = i

    	return closestWaypoint, closestWaypointIdx


    def NextWaypoint(self, waypoints, x, y, theta):
    	closestWaypoint, closestWaypointIdx = self.ClosestWaypoint(waypoints, x, y)

    	wpX = closestWaypoint.pose.pose.position.x
    	wpY = closestWaypoint.pose.pose.position.y

    	heading = math.atan2((wpY-y),(wpX-x))

    	angle = abs(theta-heading)
    	angle = min(2*math.pi - angle, angle)

    	if (angle > math.pi/4):
        	closestWaypointIdx += 1
        	if (closestWaypointIdx == len(waypoints)):
            		closestWaypointIdx = 0

	closestWaypoint = waypoints[closestWaypointIdx]
    	return closestWaypoint, closestWaypointIdx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
