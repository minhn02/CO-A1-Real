#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from visualization_msgs.msg import Marker

def main():

    rospy.init_node('box_marker_pub_node')

    name = "/box/visualization/box_body"
    pub = rospy.Publisher(name, Marker, queue_size=1)
    # name = "/box/visualization/box_cmd"
    # pub = rospy.Publisher(name, Marker, queue_size=1)

    # TF Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Get transform
        try:
            trans = tf_buffer.lookup_transform("world", "box_center", rospy.Time(0), rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rate.sleep()
            continue
        
        # Convert transform to Point message
        position = (
            trans.transform.translation.x,
            trans.transform.translation.y,
        )
        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )

        # tf.transformations.quaternion_from_euler(quaternion)

        # Publish Point
        box_marker = create_marker(position, quaternion)
        pub.publish(box_marker)

        rate.sleep()

def create_markertt(pos, quaternion):

        velocity = [1.,1.]
        m = Marker()
        
        m.header.frame_id = "/world"
        m.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        m.type = Marker.Arrow
        m.action = Marker.ADD
        m.id = 0

        # Set the scale of the marker
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.4

        # Set the color
        m.color.r = 1.0
        m.color.g = 0.75
        m.color.b = 0.0
        m.color.a = 0.75

        # Set the pose of the marker
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.2

        m.pose.orientation.x = quaternion[0]
        m.pose.orientation.y = quaternion[1]
        m.pose.orientation.z = quaternion[2]
        m.pose.orientation.w = quaternion[3]

        return m

def create_marker(pos, quaternion):

        velocity = [1.,1.]
        m = Marker()
        
        m.header.frame_id = "/world"
        m.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.id = 0

        # Set the scale of the marker
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.4

        # Set the color
        m.color.r = 1.0
        m.color.g = 0.75
        m.color.b = 0.0
        m.color.a = 0.75

        # Set the pose of the marker
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.2

        m.pose.orientation.x = quaternion[0]
        m.pose.orientation.y = quaternion[1]
        m.pose.orientation.z = quaternion[2]
        m.pose.orientation.w = quaternion[3]

        return m

if __name__ == "__main__":
    main()
