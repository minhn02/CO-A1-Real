#!/usr/bin/env python

import rospy
import tf2_ros
from codogs_plan.msg import rl_obj
import tf
from nav_msgs.msg import Odometry

has_initial_offset = False
off_x = 0
off_y = 0

def box_odometry_callback(self, msg):
    if not has_initial_offset:
        off_x = msg.pose.pose.position.x
        off_y = msg.pose.pose.position.y
        has_initial_offset = True
    box_msg = rl_ob()
    box_msg.center_x = msg.pose.pose.position.x - off_x
    dog0_msg.center_y = msg.pose.pose.position.y - off_y
    quaternion = (
        msg.pose.pose.quaternion.x,
        msg.pose.pose.quaternion.y,
        msg.pose.pose.quaternion.z,
        msg.pose.pose.quaternion.w
    )
    euler_angles0 = tf.transformations.euler_from_quaternion(quaternion0)
    yaw0 = euler_angles0[2]
    box_msg.yaw = yaw0
    box_pub.publish(box_msg)
    

def dog0_odometry_callback(self, msg):
    if has_initial_offset:
        dog_msg = rl_ob()
        dog_msg.center_x = msg.pose.pose.position.x - off_x
        dog0_msg.center_y = msg.pose.pose.position.y - off_y
        quaternion = (
            msg.pose.pose.quaternion.x,
            msg.pose.pose.quaternion.y,
            msg.pose.pose.quaternion.z,
            msg.pose.pose.quaternion.w
        )
        euler_angles0 = tf.transformations.euler_from_quaternion(quaternion0)
        yaw0 = euler_angles0[2]
        dog_msg.yaw = yaw0
        dog_p0_pub.publish(dog_msg)

def dog1_odometry_callback(self, msg):
    if has_initial_offset:
        dog_msg = rl_ob()
        dog_msg.center_x = msg.pose.pose.position.x - off_x
        dog0_msg.center_y = msg.pose.pose.position.y - off_y
        quaternion = (
            msg.pose.pose.quaternion.x,
            msg.pose.pose.quaternion.y,
            msg.pose.pose.quaternion.z,
            msg.pose.pose.quaternion.w
        )
        euler_angles0 = tf.transformations.euler_from_quaternion(quaternion0)
        yaw0 = euler_angles0[2]
        dog_msg.yaw = yaw0
        dog_p1_pub.publish(dog_msg)

def dog2_odometry_callback(self, msg):
    if has_initial_offset:
        dog_msg = rl_ob()
        dog_msg.center_x = msg.pose.pose.position.x - off_x
        dog0_msg.center_y = msg.pose.pose.position.y - off_y
        quaternion = (
            msg.pose.pose.quaternion.x,
            msg.pose.pose.quaternion.y,
            msg.pose.pose.quaternion.z,
            msg.pose.pose.quaternion.w
        )
        euler_angles0 = tf.transformations.euler_from_quaternion(quaternion0)
        yaw0 = euler_angles0[2]
        dog_msg.yaw = yaw0
        dog_p2_pub.publish(dog_msg)


def main():
    rospy.init_node('uwb_rl_translation_node')

    dogp0_pub = rospy.Publisher('dog_p0', rl_obj, queue_size=10)
    dogp1_pub = rospy.Publisher('dog_p1', rl_obj, queue_size=10)
    dogp2_pub = rospy.Publisher('dog_p2', rl_obj, queue_size=10)
    box_pub = rospy.Publisher('box_p', rl_obj, queue_size=10)
    
    box_sub = rospy.Subscriber("/box_odom", Odometry, box_odometry_callback)
    dog0_sub = rospy.Subscriber("/dog0_odom", Odometry, dog0_odometry_callback)
    dog1_sub = rospy.Subscriber("/dog1_odom", Odometry, dog1_odometry_callback)
    dog2_sub = rospy.Subscriber("/dog2_odom", Odometry, dog2_odometry_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
