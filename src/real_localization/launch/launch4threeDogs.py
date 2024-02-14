import rospy
import tf2_ros
import tf

def main():

    rospy.init_node('box_marker_pub_node')

    # TF Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform("world","box_center", rospy.Time(0), rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rate.sleep()
            continue

        off_x = trans.transform.translation.x
        off_y = trans.transform.translation.y
        context = \
        """
        <!-- -*- mode: XML -*- -->
        <launch>
        <arg name="manager" default="camera_manager"/>

        <node pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager" output="screen"/>

        <!-- Start Intel D435i -->
        <include file="$(find realsense2_camera)/launch/rs_camera.launch"/>

        <!-- RVIZ -->
        <node pkg="rviz" name="rviz" type="rviz" args="-d $(find real_localization)/rviz/rviz.rviz" output="screen"/>

        <!-- Convert to grayscale -->
        <group ns="camera/color">    
            <node pkg="image_proc" type="image_proc" name="image_proc" output="screen"/>
        </group>

        <!-- Run AprilTag detection -->
        <arg name="camera_name" default="/camera/color" />
        <arg name="image_topic" default="image_rect" />
        <arg name="queue_size" default="1" />

        <node pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="continuous_detection_node" clear_params="true" output="screen">
            <remap from="image_rect" to="$(arg camera_name)/$(arg image_topic)" />
            <remap from="camera_info" to="$(arg camera_name)/camera_info" />

            <param name="publish_tag_detections_image" type="bool" value="true" /><!-- default: false -->
            <param name="queue_size" type="int" value="$(arg queue_size)" />

            <rosparam command="load" file="$(find apriltag_ros)/config/settings.yaml"/>
            <rosparam command="load" file="$(find apriltag_ros)/config/tags.yaml"/>
        </node>

        <!-- Adjust world frame to be flat -->
        <node pkg="tf2_ros" type="static_transform_publisher" name="world_correction_transform"
            args="0 0 0 0 0 0 world_tag world " />

        <!-- static transform from T265 odometry to center of dog (s) -->
        <node pkg="tf2_ros" type="static_transform_publisher" name="robot0_pose_tf"
                args="-.06 -.06 0 -1.57 0. -1.047 camera_dog0_link dogp0" />

        <node pkg="tf2_ros" type="static_transform_publisher" name="robot1_pose_tf"
                args="0 0.02 0 3.14 -0.820 0 camera_dog1_link dogp1" />

        <node pkg="tf2_ros" type="static_transform_publisher" name="robot2_pose_tf"
                args="-0.18 0 0 0 0.820 0 camera_dog2_link dogp2" />

        <node pkg="tf2_ros" type="static_transform_publisher" name="robot3_pose_tf"
            args="-0.18 0 0 0 0.820 0 camera_dog3_link dogp3" />

        <!-- static transform from the world frame to the odom frame of the cameras(s) -->
        <!-- 0.65+.3 236  0.3-0.02+.8 813 0.22 0.80-->
        <!-- Remember to add +0.06 to x and y to account for center of dog to camera frame -->
        <node pkg="tf2_ros" type="static_transform_publisher" name="world0_frame_tf"
            args="{} {} 0 1.57 0 0 world camera_dog0_odom_frame " />

        <!-- Add .34 to x and subtract 0.2 from y -->
        <node pkg="tf2_ros" type="static_transform_publisher" name="world1_frame_tf"
            args="{} {} 0 3.14 0 0 world camera_dog1_odom_frame " />

        <node pkg="tf2_ros" type="static_transform_publisher" name="world2_frame_tf"
            args="{} {} 0 0 0 0 world camera_dog2_odom_frame " />

        <node pkg="tf2_ros" type="static_transform_publisher" name="world3_frame_tf"
            args="1.57 1.21 0 0 0 0 world camera_dog3_odom_frame " />

        <!-- Converts the tf frames to messages taken in by mappo -->
        <node pkg="real_localization" type="box_p_pub.py" name="box_p_pub_node" output="screen"/>
        <node pkg="real_localization" type="box_marker_pub.py" name="box_marker_pub_node" output="screen"/>
        <node pkg="real_localization" type="dog_p_pub.py" name="dog_p_pub_node" output="screen"/>
        <node pkg="real_localization" type="dog_marker_pub.py" name="dog_marker_pub_node" output="screen"/>
        <node pkg="real_localization" type="multi_tag_detection.py" name="multi_tag_detect_node" output="screen"/>

        <!-- Obstacle to center of obstacle -->
        <node pkg="tf2_ros" type="static_transform_publisher" name="obstacle_static_transform"
            args="0 0 -0.26 1.57 1.57 0 obstacle_tag obstacle " />
        
        </launch>
        """.format(-0.06+off_x+0.5, -0.85-0.06+off_y, 0.65+off_x-0.02, off_y, 0.18+off_x+0.5, 0.85+off_y)
        
        name_of_file = "src/real_localization/launch/localization.launch"
        file = open(name_of_file, "w")
        file.write(context)
        file.close()
        exit()


if __name__ == "__main__":
    main()