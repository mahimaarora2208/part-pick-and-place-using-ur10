#!/usr/bin/env python

from itertools import count
import rospy
import sys
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
import copy
import tf2_ros
import rosnode
from enpm809e_msgs.msg import PartInfo, PartInfos


class Navigation(object):
    """
    A controller class to drive a mobile base in Gazebo.
    """

    def __init__(self, rate=10):
        rospy.init_node('navigation_809e', anonymous=False)
        rospy.loginfo('Press Ctrl c to exit')
        rospy.Subscriber("/fiducial_transforms",
                         FiducialTransformArray, self.fiducial_transforms_cb)
        self._part_infos_pub = rospy.Publisher(
            '/part_info', PartInfos, queue_size=10, latch=True)

        self.client = actionlib.SimpleActionClient(
            'waffle/move_base', MoveBaseAction)
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # Get values from parameter server
        # Target 1
        self.marker0_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/position_x')
        self.marker0_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/position_y')
        self.o_marker0_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/orientation_x')
        self.o_marker0_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/orientation_y')
        self.o_marker0_z = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/orientation_z')
        self.o_marker0_w = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(1) + '/orientation_w')
        # Target 2
        self.marker1_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/position_x')
        self.marker1_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/position_y')
        self.o_marker1_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/orientation_x')
        self.o_marker1_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/orientation_y')
        self.o_marker1_z = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/orientation_z')
        self.o_marker1_w = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(2) + '/orientation_w')
        # Target 3
        self.marker2_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/position_x')
        self.marker2_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/position_y')
        self.o_marker2_x = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/orientation_x')
        self.o_marker2_y = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/orientation_y')
        self.o_marker2_z = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/orientation_z')
        self.o_marker2_w = rospy.get_param(
            '/aruco_lookup_locations/target_' + str(3) + '/orientation_w')

        # publish info on topic part_infos

        # Initialize  variables for publishing in part_infos topic
        # Bin a (can be any bin out of 4)
        self.bin_a = None
        self.color_a = None
        self.part_a_x = None
        self.part_a_y = None
        self.part_a_z = None
        # Bin b (can be any bin out of 4)
        self.bin_b = None
        self.color_b = None
        self.part_b_x = None
        self.part_b_y = None
        self.part_b_z = None

        # initiate a count
        self.count = 0
        # get fiducial_id
        self.fiducial_ids = set()
        self.fiducial_ids_list = []

        # rospy.loginfo("Pose for {}: [{},{},{}]"
        #                   .format(self._robot_name, self.current_x_pos, self.current_y_pos, self.current_z_pos))
        rospy.loginfo("orientation for marker 0 :{}".format(self.o_marker0_w))
        # self.start_aruco_detect()
        self.movebase_client()

    def get_transform(self, source, target):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tf_buffer)

        transform_stamped = TransformStamped()
        # Get the transform between robot_map and robot_arm_tool0

        for _ in range(5):
            try:
                transform_stamped = tf_buffer.lookup_transform(
                    source,
                    target,
                    rospy.Time(),
                    rospy.Duration(1.0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logerr("Unable to lookup transform")

        pose = Pose()
        pose.position = transform_stamped.transform.translation
        pose.orientation = transform_stamped.transform.rotation
        return pose

    def fiducial_transforms_cb(self, msg):
        if msg.transforms:
            for m in msg.transforms:
                temp_var = m.fiducial_id
                self.fiducial_ids.add(temp_var)

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.marker0_x
        goal.target_pose.pose.position.y = self.marker0_y
        goal.target_pose.pose.orientation.x = self.o_marker0_x
        goal.target_pose.pose.orientation.y = self.o_marker0_y
        goal.target_pose.pose.orientation.z = self.o_marker0_z
        goal.target_pose.pose.orientation.w = self.o_marker0_w

        self.client.send_goal(goal,
                              self.done_cb,
                              self.active_cb,
                              self.feedback_cb)

        rospy.spin()

    def active_cb(self):
        rospy.loginfo(
            "Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        if self.count == 0:
            rospy.loginfo("Moving towards target 1")
        if self.count == 1:
            rospy.loginfo("Moving towards target 2")
        if self.count == 2:
            rospy.loginfo("Moving towards target 3")

    def done_cb(self, status, result):
        """
        Callback when movebase has reached the goal

        Args:
            status (int): status of the execution
            result (str): Resut from the Action Server

        Returns:
            str: Result from the Action Server

        """

        self.count += 1  # count = 1 when target_1 is reached, count = 2 when target 2 is reached, count = 3 when target 3 is reached
        if self.count == 1:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.marker1_x
            goal.target_pose.pose.position.y = self.marker1_y
            goal.target_pose.pose.orientation.x = self.o_marker1_x
            goal.target_pose.pose.orientation.y = self.o_marker1_y
            goal.target_pose.pose.orientation.z = self.o_marker1_z
            goal.target_pose.pose.orientation.w = self.o_marker1_w
            self.client.send_goal(goal,
                                  self.done_cb,
                                  self.active_cb,
                                  self.feedback_cb)
            # self.count += 1 # count = 2

        if self.count == 2:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.marker2_x
            goal.target_pose.pose.position.y = self.marker2_y
            goal.target_pose.pose.orientation.x = self.o_marker2_x
            goal.target_pose.pose.orientation.y = self.o_marker2_y
            goal.target_pose.pose.orientation.z = self.o_marker2_z
            goal.target_pose.pose.orientation.w = self.o_marker2_w
            self.client.send_goal(goal,
                                  self.done_cb,
                                  self.active_cb,
                                  self.feedback_cb)

        # Stores part_infos values when target 2 is reached
        if len(self.fiducial_ids) == 2:
            self.fiducial_ids_list = list(self.fiducial_ids)
            rospy.loginfo(
                "Fiducial IDs Found (is list ??) : {}".format(self.fiducial_ids))
            self.bin_a = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[0]) + '/bin')
            self.color_a = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[0]) + '/part/color')
            self.part_a_x = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[0]) + '/part/location/position_x')
            self.part_a_y = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[0]) + '/part/location/position_y')
            self.part_a_z = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[0]) + '/part/location/position_z')

            self.bin_b = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[1]) + '/bin')
            self.color_b = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[1]) + '/part/color')
            self.part_b_x = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[1]) + '/part/location/position_x')
            self.part_b_y = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[1]) + '/part/location/position_y')
            self.part_b_z = rospy.get_param(
                '/kits/aruco_' + str(self.fiducial_ids_list[1]) + '/part/location/position_z')

            part1_info_msg = PartInfo()
            part1_info_msg.bin = self.bin_a
            part1_info_msg.color = self.color_a
            part1_info_msg.pose_in_bin.position.x = self.part_a_x
            part1_info_msg.pose_in_bin.position.y = self.part_a_y
            part1_info_msg.pose_in_bin.position.z = self.part_a_z

            part2_info_msg = PartInfo()
            part2_info_msg.bin = self.bin_b
            part2_info_msg.color = self.color_b
            part2_info_msg.pose_in_bin.position.x = self.part_b_x
            part2_info_msg.pose_in_bin.position.y = self.part_b_y
            part2_info_msg.pose_in_bin.position.z = self.part_b_z

            part_infos_msg = PartInfos()
            part_info_list = [part1_info_msg, part2_info_msg]
            part_infos_msg.part_infos = part_info_list
            # publishes to the topic from where manipulator will take values
            self._part_infos_pub.publish(part_infos_msg)

            rospy.loginfo("part_info_msg listtttttt :{}".format(part_info_list))

        rospy.loginfo("Fiducial IDs Found : {}".format(self.fiducial_ids))

        if status == 3 and self.count == 3:
            rospy.loginfo("Goal pose reached")
        # rosnode.kill_nodes(["aruco_detect"])

        # write code to send the robot to the next target
