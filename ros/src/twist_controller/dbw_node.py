#!/usr/bin/env python

import math
import threading
import time
import copy
import rospy
import tf

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from yaw_controller import YawController
from pid import PID
from twist_controller import GAS_DENSITY
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import numpy as np
from matplotlib import pyplot as plt

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


def rad2deg(deg):
    return deg/math.pi*180


def cte_from_waypoints(car_x, car_y, car_yaw, waypoints):
    wp_transformed_x = []
    wp_transformed_y = []
    for wp in waypoints:
        wp_x = wp.pose.pose.position.x
        wp_y = wp.pose.pose.position.y
        shift_x = wp_x - car_x
        shift_y = wp_y - car_y
        x_rot = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
        y_rot = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
        wp_transformed_x.append(x_rot)
        wp_transformed_y.append(y_rot)
    coeffs = np.polyfit(wp_transformed_x, wp_transformed_y, 3) # TODO try 5 as well
    cte = np.polyval(coeffs, .0)
    return cte


def cte_from_waypoints2(car_x, car_y, car_yaw, waypoints, n_to_fit = 8):
    wp_x = waypoints[0].pose.pose.position.x
    wp_y = waypoints[0].pose.pose.position.y
    shift_x = wp_x - car_x
    shift_y = wp_y - car_y
    # x_rot = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
    y_rot = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
    return y_rot


def unpack_pose(pose):  # TODO code duplication!
    x = pose.pose.position.x
    y = pose.pose.position.y
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return x, y, yaw


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.current_linear_velocity = .0
        self.current_yaw_velocity = .0
        self.current_velocity_lock = threading.Lock()

        self.dbw_enabled = False  # TODO good for the simulator, but what is the right value for Carla?
        self.dbw_enabled_lock = threading.Lock()

        self.twist_update_interval = 0.

        self.last_twist_cb_time = None  # Only read/write this inside twist_cb(), it is not protected by locks!

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.max_decel_torque = abs((vehicle_mass + fuel_capacity*GAS_DENSITY)*decel_limit*wheel_radius)

        self.pose_x, self.pose_y, self.pose_yaw = None, None, None
        self.pose_lock = threading.Lock()

        self.final_waypoints = []
        self.final_waypoints_lock = threading.Lock()

        self.plot_data_lock = threading.Lock()
        self.plot_cte = .0
        self.plot_velocity_err = .0
        self.plot_yaw_err = .0
        self.plot_updated = False

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=0,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

        self.throttle_controller = PID(.4, .0, .0, mn=-1., mx = accel_limit)
        ''' .1, .005, .01 '''
        # self.steering_controller = PID(.292904, .00285759, .125998, mn=-1, mx=1)
        ''' 
        pParam = .292904;
        iParam = .00285759;
        dParam = .125998;
        '''

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.DBW_enabled_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        self.loop()

    def set_plot_data(self, cte, velocity, yaw, ):
        self.plot_data_lock.acquire()
        self.plot_cte = cte
        self.plot_velocity_err = velocity
        self.plot_yaw_err = yaw
        self.plot_updated = True
        self.plot_data_lock.release()

    def get_plot_data(self):
        self.plot_data_lock.acquire()
        cte = self.plot_cte if self.plot_updated else None
        velocity = self.plot_velocity_err if self.plot_updated else None
        yaw = self.plot_yaw_err if self.plot_updated else None
        self.plot_updated = False
        self.plot_data_lock.release()
        return cte, velocity, yaw

    def set_current_velocity(self, linear, angular):
        self.current_velocity_lock.acquire()
        self.current_linear_velocity = linear
        self.current_yaw_velocity = angular
        self.current_velocity_lock.release()

    def get_current_velocity(self):
        self.current_velocity_lock.acquire()
        linear = self.current_linear_velocity
        angular = self.current_yaw_velocity
        self.current_velocity_lock.release()
        return linear, angular

    def set_dbw_enabled(self, enable):
        self.dbw_enabled_lock.acquire()
        prev_value = self.dbw_enabled
        self.dbw_enabled = enable
        if prev_value == False and enable == True:
            self.throttle_controller.reset()
        self.dbw_enabled_lock.release()

    def get_dbw_enabled(self):
        self.dbw_enabled_lock.acquire()
        enabled = self.dbw_enabled
        self.dbw_enabled_lock.release()
        return enabled

    def loop(self):
        plt.ion()
        axes = [None]*3
        fig, (axes[0], axes[1], axes[2]) = plt.subplots(nrows=3)
        n_points = 720
        x_plot = list(range(n_points))
        y_plots = []
        lines = [None]*3
        for plot_i in xrange(3):
            y_plots.append([0]*n_points)
            lines[plot_i], = axes[plot_i].plot(x_plot, y_plots[plot_i])
        plt.show()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            y1, y2, y3 = self.get_plot_data()
            if y1 is not None:
                y_to_plot = [y1, y2, y3]
                for plot_i in xrange(3):
                    del y_plots[plot_i][0]
                    y_plots[plot_i].append(y_to_plot[plot_i])
                    lines[plot_i].set_ydata(y_plots[plot_i])
                    axes[plot_i].relim()
                    axes[plot_i].autoscale_view()
                fig.canvas.draw()
                plt.pause(0.001)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def final_waypoints_cb(self, msg):
        self.final_waypoints_lock.acquire()
        self.final_waypoints = msg.waypoints
        self.final_waypoints_lock.release()

    def get_final_waypoints(self, max_n_points):
        self.final_waypoints_lock.acquire()
        waypoints = copy.deepcopy(self.final_waypoints[0:max_n_points])
        self.final_waypoints_lock.release()
        return waypoints

    def get_pose(self):
        self.pose_lock.acquire()
        pose = self.pose_x, self.pose_y, self.pose_yaw
        self.pose_lock.release()
        return pose

    def pose_cb(self, msg):
        self.pose_lock.acquire();
        self.pose_x, self.pose_y, self.pose_yaw = unpack_pose(msg)
        self.pose_lock.release();

    def twist_cb(self, msg):
        # rospy.logdebug('Received twist message:')
        # rospy.logdebug(msg)
        # self.publish(1, 0, -8)

        current_time = time.time()

        if self.last_twist_cb_time is None:
            self.last_twist_cb_time = current_time
            return

        delta_t = current_time - self.last_twist_cb_time

        if delta_t < self.twist_update_interval:
            return

        self.last_twist_cb_time = current_time

        if not self.get_dbw_enabled():
            return

        wanted_velocity = msg.twist.linear.x
        wanted_angular_velocity = msg.twist.angular.z
        current_linear_v, current_angular_v = self.get_current_velocity()

        path = self.get_final_waypoints(max_n_points=8)
        pose_x, pose_y, pose_yaw = self.get_pose()
        if len(path) > 0 and pose_x is not None:
            cte = cte_from_waypoints(pose_x, pose_y, pose_yaw, path)
        else:
            cte = 0

        steering = self.yaw_controller.get_steering(linear_velocity=wanted_velocity,
                                                    angular_velocity=wanted_angular_velocity,
                                                    current_velocity=self.current_linear_velocity)
        # linear_v_error= wanted_velocity - current_linear_v
        linear_v_error = 11.1111 - current_linear_v
        throttle = self.throttle_controller.step(linear_v_error, delta_t)
        if throttle >= 0:
            brake =.0
        else:
            brake = self.max_decel_torque * abs(throttle)
            throttle = 0

        assert 0 <= throttle <= 1
        assert 0 <= brake <= self.max_decel_torque
        self.publish(throttle=throttle, brake=brake, steer=steering)

        self.set_plot_data(cte, linear_v_error, 0)

        log_msg = 'throttle={:.4f} brake={:.4f} steer={:.4f} linear_v_error={:.4f} cte={:.4f} delta_t={:.4f} processing_time={:.4f}'
        rospy.logdebug(log_msg.format(throttle, brake, steering, linear_v_error, cte, delta_t, time.time()-current_time))

    def current_velocity_cb(self, msg):
        # rospy.logdebug('Received current velocity:')
        # rospy.logdebug(msg)
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.set_current_velocity(linear=linear, angular=angular)
        # rospy.logdebug('Set current velocity to linear={} and angular={}'.format(linear, angular))

    def DBW_enabled_cb(self, msg):
        rospy.logdebug('Received emable DBW')
        rospy.logdebug(msg)
        self.set_dbw_enabled(msg.data)


if __name__ == '__main__':
    DBWNode()

"""
-> rosmsg info geometry_msgs/TwistStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z

"""