#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from visualization_msgs.msg import Marker

def main():

    num_agent = 3

    rospy.init_node('dog_marker_pub_node')

    pubs = []
    for i in range(num_agent):
        name = "/codog{}/visualization/dog_body".format(i)
        pub = rospy.Publisher(name, Marker, queue_size=1)
        pubs.append(pub)

    # TF Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        for i in range(num_agent):
            # Get transform
            try:
                trans = tf_buffer.lookup_transform("world", "dogp{}".format(i), rospy.Time(0), rospy.Duration(1.0))
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

            # Publish Point
            dog_marker = create_marker(i, position, quaternion)
            pubs[i].publish(dog_marker)

        rate.sleep()

def create_marker(idx, pos, quaternion):

        m = Marker()
        
        m.header.frame_id = "/world"
        m.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.id = 0

        # Set the scale of the marker
        m.scale.x = 0.65
        m.scale.y = 0.3
        m.scale.z = 0.3

        # Set the color
        m.color.r = 0.5 + 0.49 * idx
        m.color.g = 0.25
        m.color.b = 0.25
        m.color.a = 1.

        # Set the pose of the marker
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.15

        m.pose.orientation.x = quaternion[0]
        m.pose.orientation.y = quaternion[1]
        m.pose.orientation.z = quaternion[2]
        m.pose.orientation.w = quaternion[3]

        return m

if __name__ == "__main__":
    main()
