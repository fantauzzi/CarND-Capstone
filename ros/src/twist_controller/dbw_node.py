#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import threading
import time
from yaw_controller import YawController
from pid import PID

from twist_controller import Controller

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


def get_cte(waypoint_x, waypoint_y, car_x, car_y, car_yaw):
    shift_x = waypoint_x - car_x
    shift_y = waypoint_y - car_y
    # x_res = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
    y_res = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
    return y_res


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

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=0,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

        self.throttle_controller = PID(.1, .005, .01, mn=.0, mx=1.)
        ''' .1, .005, .01 '''
        self.steering_controller = PID(.292904, .00285759, .125998, mn=-1, mx=1)
        ''' 
        pParam = .292904;
        iParam = .00285759;
        dParam = .125998;
        '''

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.DBW_enabled_cb)

        self.loop()

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
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
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

        '''
        steering = self.yaw_controller.get_steering(linear_velocity=wanted_velocity,
                                                    angular_velocity=wanted_angular_velocity,
                                                    current_velocity=self.current_linear_velocity)
        steering = rad2deg(steering)
        '''

        # linear_v_error= wanted_velocity - current_linear_v
        linear_v_error = 6.7056 - current_linear_v
        if linear_v_error > 0:
            throttle = self.throttle_controller.step(linear_v_error, delta_t)
        else:
            throttle = .0

        angular_v_error =  - math.tan(wanted_angular_velocity - current_angular_v) * current_linear_v * delta_t
        angular_v_error = angular_v_error ** 2 if angular_v_error > 0 else - angular_v_error ** 2
        steering = self.steering_controller.step(angular_v_error, delta_t)
        steering*=25.

        log_msg = 'Setting throttle={} and steer={} with linear_v_error={}, angular_v_error={} and delta_t={}'
        rospy.logdebug(log_msg.format(throttle, steering, linear_v_error, angular_v_error, delta_t))

        self.publish(throttle=throttle, brake=0., steer=steering)

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