#!/usr/bin/python
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf


class OdomEmitter:
    """ Emits odometry message. This is the spot rate of motion. """
    def __init__(self):
        rospy.init_node('odom_emitter')
        self.odom = rospy.Publisher('/odom', Odometry)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/x80sv/joint_states', JointState, self.handle_joint_states)
        self.wheel_base = 0.4
        self.th = 0.0
        self.x = 0.0
        self.y = 0.0

    def handle_joint_states(self, states):
        # Extract left and right velocities and emit odometry based on these facts.
        v_l, v_r = states.velocity[0], states.velocity[1]
        self.emit(v_l, v_r)

    def emit(self, v_l, v_r):
        nu = rospy.Time.now()
        dt = 0.1
        # calculate velocities:
        vx = 0.0 # We cannot move instant sideways
        vy = v_l + v_r
        vth = (v_l - v_r) / (self.wheel_base / 2.0) # Rotation

        # Integrate:
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        # Update angles:
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)

        # Send transform:
        self.odom_broadcaster.sendTransform((self.x, self.y, 0), odom_quat, nu, "odom", "map")

        # Send odom message:
        msg = Odometry()
        msg.header.stamp = nu
        msg.header.frame_id = '/odom'
        msg.child_frame_id = '/chassis'

        self.odom.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    oe = OdomEmitter()
    oe.run()
