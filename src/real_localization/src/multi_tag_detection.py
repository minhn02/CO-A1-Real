#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from visualization_msgs.msg import Marker

class MultiTagDetection:
    def __init__(self):
        self.tag_msg = None
        self.__tag_ids = [0, 1, 2, 3, 6]
        self.__tag_names = ['tag_up', 'tag_back', 'tag_left', 'tag_front', 'tag_right']
        self.__box_hwidth = 0.25
        self.__up_to_center_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.__front_to_center_quat = tf.transformations.quaternion_from_euler(1.57, 0.0, 3.14)
        self.__left_to_center_quat = tf.transformations.quaternion_from_euler(-1.5708, 1.5708, 0)
        self.__back_to_center_quat = tf.transformations.quaternion_from_euler(-1.57, 0.0, 0.0)
        self.__right_to_center_quat = tf.transformations.quaternion_from_euler(-1.5708, 0, -1.5708)

        self.__box_marker_pub = rospy.Publisher("/apriltag_box", Marker, queue_size=1)
        self.__box_center_br = tf.TransformBroadcaster()
        self.__detected_tag_sb = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.__detected_tag_cb)
        self.__box_center_timer = rospy.Timer(rospy.Duration(1/30.0), self.__box_center_cb)

        self._off_set = 0.0

    def __box_center_cb(self, event):
        if self.tag_msg:
            valid_tf = True
            if len(self.tag_msg.detections) == 0:
                valid_tf = False
                rospy.logwarn('[box detector]: no tag is detected!')
            else:
                first_detected_tag = self.tag_msg.detections[0]        
                tag_id = first_detected_tag.id[0]
                if tag_id == 0: # up
                    # print('detect 0')
                    tag_name = self.__tag_names[0]
                    self.__box_center_br.sendTransform((0.1,0,-0.2),
                                                    self.__up_to_center_quat,
                                                    rospy.Time.now(),
                                                    "box_center",
                                                    tag_name)
                elif tag_id == 1: # back
                    # rospy.logwarn('[box detector]: not support at this moment')
                    # raise NotImplementationError
                    # print('detect 1')
                    tag_name = self.__tag_names[1]
                    self.__box_center_br.sendTransform((0,0,-0.335),
                                                    self.__back_to_center_quat,
                                                    rospy.Time.now(),
                                                    "box_center",
                                                    tag_name)
                elif tag_id == 2: # left
                    # print('detect 2')
                    tag_name = self.__tag_names[2]
                    self.__box_center_br.sendTransform((0,0,-0.335),
                                                    self.__left_to_center_quat,
                                                    rospy.Time.now(),
                                                    "box_center",
                                                    tag_name)
                elif tag_id == 3: # front
                    # rospy.logwarn('[box detector]: not support at this moment')
                    # print('detect 3')
                    tag_name = self.__tag_names[3]
                    self.__box_center_br.sendTransform((0,0,-0.335),
                                                    self.__front_to_center_quat,
                                                    rospy.Time.now(),
                                                    "box_center",
                                                    tag_name)
                elif tag_id == 6: # right
                    rospy.logwarn('[box detector]: not support at this moment')
                    # print('detect 6')
                    tag_name = self.__tag_names[4]
                    self.__box_center_br.sendTransform((0,0,-self.__box_hwidth),
                                                    self.__right_to_center_quat,
                                                    rospy.Time.now(),
                                                    "box_center",
                                                    tag_name)
                else:
                    valid_tf = False
                    rospy.logwarn('[box detector]: tag number is wrong!')
            
            if valid_tf:
                self.publish_box_marker()

    def __detected_tag_cb(self, msg):
        self.tag_msg = msg

    def publish_box_marker(self):
        marker = Marker()

        marker.header.frame_id = "apriltag_box"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = marker.CUBE
        marker.id = 0
        marker.action = marker.ADD

        # Set the scale of the marker
        marker.scale.x = self.__box_hwidth * 2.0
        marker.scale.y = self.__box_hwidth * 2.0
        marker.scale.z = self.__box_hwidth * 2.0

        # Set the color
        marker.color.r = 255.0/255.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.__box_marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("multi_tag_detection")
    mtd = MultiTagDetection()
    rospy.spin()