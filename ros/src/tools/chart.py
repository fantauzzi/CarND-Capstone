#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from matplotlib import pyplot as plt
from threading import Lock
import copy


class Chart(object):
    def __init__(self):
        rospy.init_node('chart', log_level=rospy.DEBUG)
        rospy.logdebug('Tools runing')

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.waypoints = []
        self.fig, self.ax = plt.subplots()
        self.waypoints_lock = Lock()
        self.pose = None
        self.is_pose_updated = False
        self.pose_lock = Lock()

        self.loop()

    def get_waypoints(self):
        self.waypoints_lock.acquire()
        wps = self.waypoints
        self.waypoints_lock.release()
        return wps

    def pose_updated(self):
        self.pose_lock.acquire()
        result = self.is_pose_updated
        self.pose_lock.release()
        return result

    def get_pose(self):
        self.pose_lock.acquire()
        result = copy.deepcopy(self.pose)
        self.is_pose_updated = False
        self.pose_lock.release()
        return result

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        waypoints=[]
        while not rospy.is_shutdown() and len(waypoints) == 0:
            waypoints = self.get_waypoints()
            rate.sleep()
        plt.ion()
        x, y = [], []
        for waypoint in waypoints:
            x.append(waypoint.pose.pose.position.x)
            y.append(waypoint.pose.pose.position.y)
        self.ax.plot(x, y)
        plt.show()
        while not rospy.is_shutdown():
            if self.pose_updated():
                x, y = self.get_pose()
                self.ax.plot(x, y, 'o')
                self.fig.canvas.draw()
            rate.sleep()

    def waypoints_cb(self, waypoints):
        # DONE: Implement
        assert len(self.waypoints) == 0
        self.waypoints_lock.acquire()
        self.waypoints = waypoints.waypoints
        self.waypoints_lock.release()
        rospy.logdebug('Received {} waypoints:'.format(len(waypoints.waypoints)))
        # rospy.logdebug(waypoints)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.pose = x, y
        self.is_pose_updated = True
        self.pose_lock.release()


if __name__ == '__main__':
    try:
        Chart()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

