#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from scipy.spatial.distance import euclidean

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5
STOPLINE_BUFFER_WPS = 2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # DONE: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None
        self.kd_tree = None # kd tree to conduct nearest neighbor search
        self.base_lane = None
        self.stopline_wp_idx = -1

        self.loop()

    def loop(self):
        # publishing loop
        rate = rospy.Rate(50) # 50hz
        while not rospy.is_shutdown():
            if None not in (self.current_pose, self.kd_tree, self.lane):
                final_lane = self.generate_lane()
                self.final_waypoints_pub.publish(final_lane)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # get index of waypoint closest to the car
        closest_waypoint_idx = self.kd_tree.query(self.current_pose)[1]
        # check if point is behind or ahead of vehicle by comparing distance to previous point
        previous_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx - 1].pose.pose.position.y]
        current_waypoint = [self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.x, self.base_waypoints.waypoints[closest_waypoint_idx].pose.pose.position.y]
        car_dist = euclidean(self.current_pose, previous_waypoint)
        waypoint_dist = euclidean(current_waypoint, previous_waypoint)
        # if the car is further away from the previous waypoint than the closest waypoint, then the closest waypoint is behind the car and we should take the next waypoint in the list
        if car_dist > waypoint_dist:
            closest_waypoint_idx += 1        
        return closest_waypoint_idx
    
    def generate_lane(self):
        lane = Lane()     
        # publish list of base waypoints starting from the waypoint closest to the car
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        # modify base waypoints when red light in range
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate(base_waypoints, closest_idx)
        return lane

    def decelerate(self, waypoints, closest_idx):
        modified_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # the car should stop a distance before the stopline
            stop_idx = max(self.stopline_wp_idx - closest_idx - STOPLINE_BUFFER_WPS, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            vel = min(vel, self.get_waypoint_velocity(self, p))
            self.set_waypoint_velocity(self, p, vel)
            modified_waypoints.append(p)
        return modified_waypoints

    def pose_cb(self, msg):
        ## called at 50 hz
        ## contains position x,y and quaternion z,w components
        # save current [x,y] position
        self.current_pose = [msg.pose.position.x, msg.pose.position.y]

    def waypoints_cb(self, lane):
        ## called once on startup
        # save base waypoints as lane object
        self.base_waypoints = lane
        # create list of [x,y] waypoint positions to initialize kd_tree
        kd_tree_pts = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]
        self.kd_tree = KDTree(kd_tree_pts)

    def traffic_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

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
