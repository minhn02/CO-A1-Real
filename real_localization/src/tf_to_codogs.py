#!/usr/bin/env python

import rospy
import tf2_ros
from codogs_plan.msg import rl_obj
import tf

def main():
    rospy.init_node('tf_to_rl_msg_publisher')

    # TF Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    dogp0_pub = rospy.Publisher('dog_p0', rl_obj, queue_size=10)
    box_p_pub = rospy.Publisher('box_p', rl_obj, queue_size=10)

    rate = rospy.Rate(100)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get transform
            try:
                trans = tf_buffer.lookup_transform("world", "dogp0", rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            
            # Convert transform to Point message
            dogp0_msg = rl_obj()
            dogp0_msg.center_x = trans.transform.translation.x
            dogp0_msg.center_y = trans.transform.translation.y
            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )

            euler_angles = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler_angles[2]
            dogp0_msg.yaw = yaw

            # Publish Point
            dogp0_pub.publish(dogp0_msg)

            try:
                trans = tf_buffer.lookup_transform("world", "box_p", rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            
            # Convert transform to Point message
            boxp_msg = rl_obj()
            boxp_msg.center_x = trans.transform.translation.x
            boxp_msg.center_y = trans.transform.translation.y
            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )

            euler_angles = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler_angles[2]
            boxp_msg.yaw = yaw

            # Publish Point
            box_p_pub.publish(boxp_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            continue

        rate.sleep()

if __name__ == "__main__":
    main()
