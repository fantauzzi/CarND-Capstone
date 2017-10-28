#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import tf
import math
import threading

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

def unpack_pose(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    return x, y, z


def poses_distance(pose1, pose2):
    x1, y1, z1 = unpack_pose(pose1)
    x2, y2, z2 = unpack_pose(pose2)
    distance = ((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)**.5
    return distance


def get_closest_waypoint_idx(pose, waypoints):
    min_dist = sys.float_info.max
    min_dist_i = None
    for wp_i in xrange(len(waypoints)):
        waypoint_pose=waypoints[wp_i].pose.pose
        dist = poses_distance(pose, waypoint_pose)
        if dist < min_dist:
            min_dist = dist
            min_dist_i = wp_i
    return min_dist_i


def get_bearing_from_pose(my_pose, from_pose):
    my_x, my_y, _ = unpack_pose(my_pose)
    from_x, from_y, _ = unpack_pose(from_pose)
    bearing = math.atan2(from_y-my_y, from_x-my_x)
    return bearing


def get_next_waypoint_idx(pose, waypoints):
    wp_i = get_closest_waypoint_idx(pose, waypoints)
    assert wp_i >= 0
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, pose_yaw = tf.transformations.euler_from_quaternion(quaternion)
    bearing = get_bearing_from_pose(pose, waypoints[wp_i].pose.pose)
    if abs(bearing-pose_yaw) > math.pi / 4:  # TODO Should it be math.pi / 2 ?
        wp_i = (wp_i + 1) % len(waypoints)
    return wp_i


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        rospy.logdebug('Inside WaypointUpdater.__init__()')
        rospy.logdebug('Running Python version '+sys.version)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints= None

        self.received_pose_count = 0
        self.lock = threading.Lock()

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # rospy.logdebug("Inside pose_cb()")
        self.lock.acquire();
        current_count = self.received_pose_count
        self.received_pose_count += 1
        self.lock.release();
        # if current_count % 25 != 0:
        #    return

        rospy.logdebug('Received pose #{}'.format(current_count))
        rospy.logdebug(msg)
        pose_i = get_next_waypoint_idx(msg.pose, self.waypoints)
        rospy.logdebug('Next waypoint is #{}'.format(pose_i))
        next_i = (pose_i+1) % len(self.waypoints)
        prev_i = (pose_i-1) % len(self.waypoints)
        dist_next_i = poses_distance(msg.pose, self.waypoints[next_i].pose.pose)
        dist_prev_i = poses_distance(msg.pose, self.waypoints[prev_i].pose.pose)
        direction = 1 if dist_next_i < dist_prev_i else -1
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        # lane.waypoints = self.waypoints[pose_i: pose_i+direction*200: direction]
        for count in xrange(LOOKAHEAD_WPS):
            i = (pose_i+count*direction) % len(self.waypoints)
            lane.waypoints.append(self.waypoints[i])
        self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        assert self.waypoints is None
        self.waypoints= waypoints.waypoints
        rospy.logdebug('Received {} waypoints:'.format(len(waypoints.waypoints)))
        # rospy.logdebug(waypoints)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)


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
