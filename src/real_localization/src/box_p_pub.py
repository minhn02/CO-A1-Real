#!/usr/bin/env python

import rospy
import tf2_ros
from codogs_plan.msg import rl_obj
import tf

def main():
    rospy.init_node('box_p_pub_node')

    # TF Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    box_p_pub = rospy.Publisher('box_p', rl_obj, queue_size=10)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Get transform
        try:
            trans = tf_buffer.lookup_transform("world", "box_p", rospy.Time(0), rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rate.sleep()
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

        rate.sleep()

if __name__ == "__main__":
    main()
