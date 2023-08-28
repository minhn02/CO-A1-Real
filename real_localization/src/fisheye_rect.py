#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters

bridge = CvBridge()

def callback(image_msg, camera_info):
    global bridge

    # Convert the image from ROS to OpenCV format
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Could not convert image: %s", e)
        return

    # Extract the camera intrinsic parameters from the CameraInfo message
    K = np.array(camera_info.K).reshape(3, 3)
    D = np.array(camera_info.D)

    # Perform fisheye rectification
    h, w = cv_image.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)
    undistorted_img = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Publish the undistorted image
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(undistorted_img, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr("Could not publish image: %s", e)

if __name__ == '__main__':
    rospy.init_node('t265_rectify', anonymous=True)
    image_pub = rospy.Publisher("/camera/fisheye1/image_rect", Image, queue_size=10)

    image_sub = message_filters.Subscriber("/camera/fisheye1/image_raw", Image)
    info_sub = message_filters.Subscriber("/camera/fisheye1/camera_info", CameraInfo)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
    ts.registerCallback(callback)

    rospy.loginfo("T265 fisheye rectification node has started.")
    rospy.spin()
