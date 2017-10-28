#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import tf
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

def unpack_waypoint_position(wp):
    x = wp.pose.pose.position.x
    y = wp.pose.pose.position.y
    z = wp.pose.pose.position.z
    return x, y, z


def wp_distance(wp1, wp2):
    x1, y1, z1 = unpack_waypoint_position(wp1)
    x2, y2, z2 = unpack_waypoint_position(wp2)
    distance = ((x1+x2)**2+(y1+y2)**2+(z1+z2)**2)**.5
    return distance

def get_closest_waypoint_idx(pose, waypoints):
    min_dist = sys.float_info.max
    min_dist_i = None
    for wp, wp_i in enumerate(waypoints):
        dist = wp_distance(pose, wp)
        if dist < min_dist:
            min_dist = dist
            min_dist_i = wp_i
    return min_dist_i

def get_waypoint_bearing(pose, wp):
    pose_x, pose_y, _ = unpack_waypoint_position(pose)
    wp_x, wp_y, _ = unpack_waypoint_position(wp)
    bearing = math.atan2(wp_y-pose_y, wp_x-pose_x)
    return bearing

def get_next_waypoint_idx(pose, waypoints):
    wp_i = get_closest_waypoint_idx(pose, waypoints)
    assert wp_i >= 0
    _, _, pose_yaw = tf.transformers.euler_from_quaternion(pose)
    bearing = get_waypoint_bearing(pose, waypoints[wp_i])
    if abs(bearing-pose_yaw) < math.pi / 4:  # TODO Should it be math.pi / 2 ?
        wp_i += 1
        if wp_i >= len(waypoints):
            wp_i -= len(waypoints)
            assert wp_i == 0
    return wp_i

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.logdebug('Inside WaypointUpdater.__init__()')
        rospy.logdebug('Running Python version '+sys.version)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints= None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.waypoints is not None:
            rospy.logdebug('List of waypoints supposed to be empty but it already contains {} elements'.format(len(self.waypoints)))
        self.waypoints= waypoints
        rospy.logdebug('Received these {} waypoints:'.format(len(waypoints.waypoints)))
        rospy.logdebug(waypoints)
        """
        From waypoint_loader.py:
        
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity)
        """

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
