#!/usr/bin/env python

import rospy
import tf2_ros
from codogs_plan.msg import rl_obj
import tf

def main():
    rospy.init_node('dog_p_pub_node')

    # TF Buffer and Listener
    tf_buffer0 = tf2_ros.Buffer()
    tf_listener0 = tf2_ros.TransformListener(tf_buffer0)

    tf_buffer1 = tf2_ros.Buffer()
    tf_listener1 = tf2_ros.TransformListener(tf_buffer1)

    tf_buffer2 = tf2_ros.Buffer()
    tf_listener2 = tf2_ros.TransformListener(tf_buffer2)

    tf_buffer3 = tf2_ros.Buffer()
    tf_listener3 = tf2_ros.TransformListener(tf_buffer3)

    dogp0_pub = rospy.Publisher('dog_p0', rl_obj, queue_size=10)
    dogp1_pub = rospy.Publisher('dog_p1', rl_obj, queue_size=10)
    dogp2_pub = rospy.Publisher('dog_p2', rl_obj, queue_size=10)
    dogp3_pub = rospy.Publisher('dog_p3', rl_obj, queue_size=10)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Get transform
        try:
            trans0 = tf_buffer0.lookup_transform("world", "dogp0", rospy.Time(0), rospy.Duration(1.0))
            trans1 = tf_buffer1.lookup_transform("world", "dogp1", rospy.Time(0), rospy.Duration(1.0))
            trans2 = tf_buffer2.lookup_transform("world", "dogp2", rospy.Time(0), rospy.Duration(1.0))
            trans3 = tf_buffer3.lookup_transform("world", "dogp3", rospy.Time(0), rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rate.sleep()
            continue
        
        # Convert transform to Point message
        dogp0_msg = rl_obj()
        dogp0_msg.center_x = trans0.transform.translation.x
        dogp0_msg.center_y = trans0.transform.translation.y
        quaternion0 = (
            trans0.transform.rotation.x,
            trans0.transform.rotation.y,
            trans0.transform.rotation.z,
            trans0.transform.rotation.w
        )

        euler_angles0 = tf.transformations.euler_from_quaternion(quaternion0)
        yaw0 = euler_angles0[2]
        dogp0_msg.yaw = yaw0

        # Publish Point
        dogp0_pub.publish(dogp0_msg)

        dogp1_msg = rl_obj()
        dogp1_msg.center_x = trans1.transform.translation.x
        dogp1_msg.center_y = trans1.transform.translation.y
        quaternion1 = (
            trans1.transform.rotation.x,
            trans1.transform.rotation.y,
            trans1.transform.rotation.z,
            trans1.transform.rotation.w
        )

        euler_angles1 = tf.transformations.euler_from_quaternion(quaternion1)
        yaw1 = euler_angles1[2]
        dogp1_msg.yaw = yaw1

        # Publish Point
        dogp1_pub.publish(dogp1_msg)

        dogp2_msg = rl_obj()
        dogp2_msg.center_x = trans2.transform.translation.x
        dogp2_msg.center_y = trans2.transform.translation.y
        quaternion2 = (
            trans2.transform.rotation.x,
            trans2.transform.rotation.y,
            trans2.transform.rotation.z,
            trans2.transform.rotation.w
        )

        euler_angles2 = tf.transformations.euler_from_quaternion(quaternion2)
        yaw2 = euler_angles2[2]
        dogp2_msg.yaw = yaw2

        # Publish Point
        dogp2_pub.publish(dogp2_msg)

        dogp3_msg = rl_obj()
        dogp3_msg.center_x = trans3.transform.translation.x
        dogp3_msg.center_y = trans3.transform.translation.y
        quaternion3 = (
            trans3.transform.rotation.x,
            trans3.transform.rotation.y,
            trans3.transform.rotation.z,
            trans3.transform.rotation.w
        )

        euler_angles3 = tf.transformations.euler_from_quaternion(quaternion3)
        yaw3 = euler_angles3[2]
        dogp3_msg.yaw = yaw3

        # Publish Point
        dogp3_pub.publish(dogp3_msg)
        
        rate.sleep()

if __name__ == "__main__":
    main()
