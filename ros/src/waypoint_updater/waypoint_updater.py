#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped , TwistStamped
from styx_msgs.msg import Lane, Waypoint , TrafficLightArray , TrafficLight
from std_msgs.msg import Int32, Float32
import numpy as np
import math
import tf
from threading import Thread, Lock
from copy import deepcopy
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
TAFFIC_ACTIVATED = False

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        # rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # self.mutex = Lock()

        self.base_waypoints = None
        self.current_pose = None
        self.next_waypoint_id = None
        self.traffic_light_waypoint_id = None
        self.obstacle_waypoint_id = None

        self.deceleration = 1
        self.tl_stop_buffer = 10.0  # safe buffer distance before traffic light in meters


        # Loop Event for updating final_waypoints
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.waypoints_updater()
            rate.sleep()

    def waypoints_updater(self):
        if self.base_waypoints and self.current_pose:
            self.next_waypoint_id = self.get_next_waypoint(self.current_pose, self.base_waypoints)
            # if self.next_waypoint_id:
            #     rospy.loginfo("next waypoint id = %d" , self.next_waypoint_id)
            final_waypoints = self.get_final_waypoints(self.base_waypoints.waypoints, self.next_waypoint_id,
                                                       self.next_waypoint_id+LOOKAHEAD_WPS,self.traffic_light_waypoint_id)

            lane = Lane()
            lane.header.stamp = rospy.Time().now()
            lane.header.frame_id = '/world'
            lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(lane)

    def nearest_waypoint(self,x,y,waypoints_list):
        min_dist = float('inf')
        nearest_point_id = -1
        for id , waypoint in enumerate(waypoints_list.waypoints):
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y

            dist = np.sqrt((waypoint_x-x)**2 + (waypoint_y-y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_point_id = id

        return nearest_point_id

    def get_next_waypoint(self,current_pose,waypoints_list):

        current_pose_x = current_pose.pose.position.x
        current_pose_y = current_pose.pose.position.y
        nearest_point_id = self.nearest_waypoint(current_pose_x,current_pose_y,waypoints_list)
        nearest_point_x = waypoints_list.waypoints[nearest_point_id].pose.pose.position.x
        nearest_point_y = waypoints_list.waypoints[nearest_point_id].pose.pose.position.y
        heading = np.arctan2((nearest_point_y-current_pose_y) , (nearest_point_x-current_pose_x))

        x = current_pose.pose.orientation.x
        y = current_pose.pose.orientation.y
        z = current_pose.pose.orientation.z
        w = current_pose.pose.orientation.w
        euler_angles_xyz = tf.transformations.euler_from_quaternion([x, y, z, w])
        theta = euler_angles_xyz[-1]
        angle = math.fabs(theta - heading)
        if angle > math.pi / 4.0:
            nearest_point_id += 1

        return nearest_point_id

    def get_final_waypoints(self, waypoints, start_wp, end_wp, tf_waypoint_id):

        final_waypoints = []
        if tf_waypoint_id != -1 and tf_waypoint_id in range(start_wp,end_wp,1):

            for i in range(start_wp, tf_waypoint_id):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y = waypoints[index].pose.pose.position.y
                wp.pose.pose.position.z = waypoints[index].pose.pose.position.z
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                dist = self.distance(waypoints, i, tf_waypoint_id)
                if dist < self.tl_stop_buffer:
                    wp.twist.twist.linear.x = 0.0
                else:
                    wp.twist.twist.linear.x = min(waypoints[index].twist.twist.linear.x,
                                                  math.sqrt(2 * self.deceleration * max(0.0,dist-self.tl_stop_buffer)))


                final_waypoints.append(wp)

            for i in range(tf_waypoint_id, end_wp):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y = waypoints[index].pose.pose.position.y
                wp.pose.pose.position.z = waypoints[index].pose.pose.position.z
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                wp.twist.twist.linear.x = 0
                final_waypoints.append(wp)
        else:
            for i in range(start_wp, end_wp):
                index = i % len(waypoints)
                wp = Waypoint()
                wp.pose.pose.position.x = waypoints[index].pose.pose.position.x
                wp.pose.pose.position.y = waypoints[index].pose.pose.position.y
                wp.pose.pose.position.z = waypoints[index].pose.pose.position.z
                wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
                final_waypoints.append(wp)

        return final_waypoints

    def pose_cb(self, msg):

        self.current_pose = msg

    def waypoints_cb(self, waypoints):

        self.base_waypoints = waypoints

    def traffic_cb(self, msg):

        self.traffic_light_waypoint_id = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint_id = msg.data

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
