#!/usr/bin/env python3


# python
from itertools import count
import sys
import copy

from numpy import place
# ros
import rospy
import tf
from tf.transformations import euler_from_quaternion
from enpm809e_msgs.msg import LogicalCameraImage
from enpm809e_msgs.msg import PartInfos
from geometry_msgs.msg import Pose
from enpm809e_msgs.srv import VacuumGripperControl
from enpm809e_msgs.msg import VacuumGripperState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# moveit
import moveit_commander as mc
import moveit_msgs.msg


class Manipulation(object):
    def __init__(self, node_name='manipulation_809e', ns='',
                 robot_description='robot_description'):

        self.joint_publisher = rospy.Publisher(
            "/ariac/kitting/kitting_arm_controller/command",
            JointTrajectory,
            queue_size=100)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        self._tf_listener = tf.TransformListener()
        self.list_of_parts_cam1 = []
        self.list_of_parts_cam2 = []
        self.concatenated_list_of_parts = []
        self.color_of_parts_to_place = []
        self.bin_of_parts_to_place = []
        self.final = []
        self.cam_name_1 = ''
        self.cam_name_2 = ''
        self.pick_coordinates_a = []
        self.pick_coordinates_b = []
        self.place_a = []
        self.place_b = []
        self.count = 0
        self.id_part = None

        # kitting_arm
        # - linear_arm_actuator_joint
        # - shoulder_pan_joint
        # - shoulder_lift_joint
        # - elbow_joint
        # - wrist_1_joint
        # - wrist_2_joint
        # - wrist_3_joint

        # Get info from topic /part_info
        msg_part_info = rospy.wait_for_message(
            '/part_info', PartInfos)
        # Dictionary to store values from part info (goal locations)
        if msg_part_info.part_infos:
            number_of_parts_to_locate = len(msg_part_info.part_infos)
            part_a = msg_part_info.part_infos[0]
            part_b = msg_part_info.part_infos[1]
            self.part_a_info = {'bin': part_a.bin, 'color': part_a.color, 'position_x': part_a.pose_in_bin.position.x,
                                'position_y': part_a.pose_in_bin.position.y, 'position_z': part_a.pose_in_bin.position.z, 'orientation_x': part_a.pose_in_bin.orientation.x, 'orientation_y': part_a.pose_in_bin.orientation.y, 'orientation_z': part_a.pose_in_bin.orientation.z, 'orientation_w': part_a.pose_in_bin.orientation.w}
            # rospy.loginfo("msg info PART A {}".format(self.part_a_info))
            self.part_b_info = {'bin': part_b.bin, 'color': part_b.color, 'position_x': part_b.pose_in_bin.position.x,
                                'position_y': part_b.pose_in_bin.position.y, 'position_z': part_b.pose_in_bin.position.z, 'orientation_x': part_b.pose_in_bin.orientation.x, 'orientation_y': part_b.pose_in_bin.orientation.y, 'orientation_z': part_b.pose_in_bin.orientation.z, 'orientation_w': part_b.pose_in_bin.orientation.w}

            # creating a list of parts to pick and place from aruco
            # Stores color of parts in a list
            if self.part_a_info['color'] and self.part_b_info['color']:
                self.color_of_parts_to_place.append(self.part_a_info['color'])
                self.color_of_parts_to_place.append(self.part_b_info['color'])
            rospy.loginfo("colors from kitting {}".format(
                self.color_of_parts_to_place))

            # Stores goal bin location for parts
            if self.part_a_info['bin'] and self.part_b_info['bin']:
                self.bin_of_parts_to_place.append(self.part_a_info['bin'])
                self.bin_of_parts_to_place.append(self.part_b_info['bin'])
            rospy.loginfo("bin from kitting {}".format(
                self.bin_of_parts_to_place))

        # To save part info from camera
        # Camera 1
        msg_camera_1_part_info = rospy.wait_for_message(
            '/logical_camera/logical_camera_1', LogicalCameraImage)
        if msg_camera_1_part_info:
            number_of_parts_under_camera1 = len(msg_camera_1_part_info.models)
            if number_of_parts_under_camera1 == 2:
                cam1_part_a = msg_camera_1_part_info.models[0]
                cam1_part_b = msg_camera_1_part_info.models[1]
                self.list_of_parts_cam1 = [cam1_part_a.type, cam1_part_b.type]
            elif number_of_parts_under_camera1 == 1:
                cam1_part_a = msg_camera_1_part_info.models[0]
                self.list_of_parts_cam1 = [cam1_part_a.type]
            else:
                rospy.loginfo("No part under Camera 1")

        # rospy.loginfo("camera1 assem value {}"
        #               .format(self.list_of_parts_cam1))

        # Camera 2
        msg_camera_2_part_info = rospy.wait_for_message(
            '/logical_camera/logical_camera_2', LogicalCameraImage)
        if msg_camera_2_part_info:
            number_of_parts_under_camera2 = len(msg_camera_2_part_info.models)
            if number_of_parts_under_camera2 == 2:
                cam2_part_a = msg_camera_2_part_info.models[0]
                cam2_part_b = msg_camera_2_part_info.models[1]
                self.list_of_parts_cam2 = [cam2_part_a.type, cam2_part_b.type]
            elif number_of_parts_under_camera2 == 1:
                cam2_part_a = msg_camera_2_part_info.models[0]
                self.list_of_parts_cam2 = [cam2_part_a.type]
            else:
                rospy.loginfo("No part under Camera 2")

        # dictionary to store pre-set locations
        self.locations = {}

        # rospy.loginfo("msg info : {}".format(self.goal_locations))

        name = 'home'
        arm_joints = [0, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self.locations[name] = (arm_joints)

        name = 'test'
        arm_joints = [1, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self.locations[name] = (arm_joints)

        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)

        moveit_group = mc.MoveGroupCommander(
            'kitting_arm', robot_description=ns + '/' + robot_description, ns=ns)
        self.groups = {}
        self.groups['kitting_arm'] = moveit_group
        self._arm_group = self.groups['kitting_arm']
        self._arm_group.set_goal_orientation_tolerance = 0.051
        self._arm_group.set_goal_position_tolerance = 0.051

        # rospy.logerr(self.groups['kitting_arm'].get_current_pose())
        # ee_link
        # rospy.logerr(self.groups['kitting_arm'].get_end_effector_link())
        self.pickup_pose_1 = None

    def main(self):
        """
        Main function to start the Node core
        """
        # Instantiate camera type to concatenate for frames
        cam_name_1 = ''
        cam_name_2 = ''
        # To get total parts in the workcell
        self.concatenated_list_of_parts = self.list_of_parts_cam1 + self.list_of_parts_cam2

        # Checks under which camera is the part present and assigns the camera type
        if self.color_of_parts_to_place[0][0] != self.color_of_parts_to_place[1][0]:
            if self.list_of_parts_cam1:
                if(([s for s in self.list_of_parts_cam1 if self.color_of_parts_to_place[0] in s])):
                    cam_name_1 = "logical_camera_1_"
                if(([s for s in self.list_of_parts_cam1 if self.color_of_parts_to_place[1] in s])):

                    cam_name_2 = "logical_camera_1_"

            if self.list_of_parts_cam2:
                if(([s for s in self.list_of_parts_cam2 if self.color_of_parts_to_place[0] in s])):
                    cam_name_1 = "logical_camera_2_"
                if(([s for s in self.list_of_parts_cam2 if self.color_of_parts_to_place[1] in s])):
                    cam_name_2 = "logical_camera_2_"

            self.final = [cam_name_1 + "assembly_pump_"+self.color_of_parts_to_place[0] + "_0_frame",
                          cam_name_2 + "assembly_pump_" + self.color_of_parts_to_place[1] + "_0_frame"]

        else:  # goes here when both parts are same
            num = 0
            for ele in self.concatenated_list_of_parts:
                a = ele.find(self.color_of_parts_to_place[0])
                if a == -1:  # if element not found in colors to place list
                    flag = False
                else:
                    num += 1   # count number of same parts

            # if one part manipulation is needed (blue -> bin2, same blue -> bin 3)
            if num == 1:
                self.id_part = "_0_frame"
                if self.list_of_parts_cam1:
                    # checks if part is in color list
                    if(([s for s in self.list_of_parts_cam1 if self.color_of_parts_to_place[0] in s])):
                        cam_name_1 = "logical_camera_1_"
                        # if part is in bin 1 or bin 2
                        if(([s for s in self.bin_of_parts_to_place[0] if ("bin1" or "bin2") in s])):
                            cam_name_2 = "logical_camera_1_"
                        else:
                            cam_name_2 = "logical_camera_2_"

                if self.list_of_parts_cam2:
                    if(([s for s in self.list_of_parts_cam2 if self.color_of_parts_to_place[0] in s])):
                        cam_name_1 = "logical_camera_2_"
                        if(([s for s in self.bin_of_parts_to_place[0] if ("bin1" or "bin2") in s])):
                            cam_name_2 = "logical_camera_1_"
                        else:
                            cam_name_2 = "logical_camera_2_"

            # when two parts of same color are present. This checks for their locations
            else:
                if self.list_of_parts_cam1:
                    if(([s for s in self.list_of_parts_cam1 if self.color_of_parts_to_place[0] in s])):
                        cam_name_1 = "logical_camera_1_"
                        num = 0
                        for ele in self.list_of_parts_cam1:
                            a = ele.find(self.color_of_parts_to_place[0])
                            if a == -1:
                                flag = False
                            else:
                                num += 1

                        if num == 1:
                            cam_name_2 = "logical_camera_2_"

                        else:
                            cam_name_2 = "logical_camera_1_"  # TO-DO TEST WITH DIFF VALUES

                if self.list_of_parts_cam2:
                    if(([s for s in self.list_of_parts_cam2 if self.color_of_parts_to_place[0] in s])):
                        cam_name_1 = "logical_camera_2_"
                        num = 0
                        for ele in self.list_of_parts_cam1:
                            a = ele.find(self.color_of_parts_to_place[0])
                            if a == -1:
                                flag = False
                            else:
                                num += 1

                        if num == 1:
                            cam_name_2 = "logical_camera_2_"
                        else:
                            cam_name_2 = "logical_camera_1_"
                self.id_part = "_1_frame"

            self.final = [cam_name_1 + "assembly_pump_"+self.color_of_parts_to_place[0] + "_0_frame",
                          cam_name_2 + "assembly_pump_" + self.color_of_parts_to_place[1] + self.id_part]

        k = 0
        while k < 3:
            self.pick_coordinates_a = self.get_transform(
                "/world", str(self.final[0]))
            k += 1

        self.place_a = self.broadcast_marker(
            self.part_a_info['position_x'],  self.part_a_info['position_y'],  self.part_a_info['position_z'], 0, 0, 0, 1, "part_a", str(self.bin_of_parts_to_place[0]))
        # pick-and-place
        if self.pick_coordinates_a and self.place_a:
            self.pickandplace()
        else:
            rospy.loginfo(
                "One of the required values is None due to broadcaster issue....Run again")

    def get_transform(self, parent_frame, child_frame):

        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(parent_frame,
                                               child_frame,
                                               now, rospy.Duration(0.5)
                                               )
            (trans, rot) = self._tf_listener.lookupTransform(
                parent_frame, child_frame, now)
            pick_coordinates = trans
            return pick_coordinates

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")

    def broadcast_marker(self, x, y, z, x_q, y_q, z_q, w_q, child, parent):
        br = tf.TransformBroadcaster()
        place_coordinates_from_broadcaster = None
        k = 0
        while k < 2:
            br.sendTransform((x, y, z),
                             (x_q, y_q, z_q, w_q),
                             rospy.Time.now(),
                             child,
                             parent)
            j = 0
            while j < 1:
                place_coordinates_from_broadcaster = self.get_transform(
                    "world", str(child))
                j += 1
            # rospy.sleep(0.1)
            k += 1

        rospy.loginfo("insideeeeeee valuee {}".format(
            place_coordinates_from_broadcaster))
        return place_coordinates_from_broadcaster

    def reach_goal(self):
        """
        Give a goal to the end effector to reach
        """
        rospy.loginfo("current_pose {}".format(
            self._arm_group.get_current_pose()))
        pose_to_reach = copy.deepcopy(self._arm_group.get_current_pose())
        pose_to_reach.pose.position.x -= 1
        pose_to_reach.pose.position.z += 0.5
        rospy.loginfo("pose_to_go {}".format(pose_to_reach))
        self._arm_group.set_pose_target(pose_to_reach)
        self._arm_group.go()

    def publish_joint_values(self, duration=0.1):
        """
        Publish joint values to the Topic /ariac/kitting/kitting_arm_controller/command
        """
        joint_values = [-1, 0.09, -0.75, 2.02, -2.11, -1.58, 0]

        jt_ur10 = JointTrajectory()
        jt_ur10.joint_names = ['linear_arm_actuator_joint',
                               'shoulder_pan_joint',
                               'shoulder_lift_joint',
                               'elbow_joint',
                               'wrist_1_joint',
                               'wrist_2_joint',
                               'wrist_3_joint']

        jtpt = JointTrajectoryPoint()
        jtpt.positions = [joint_values[0],
                          joint_values[1],
                          joint_values[2],
                          joint_values[3],
                          joint_values[4],
                          joint_values[5],
                          joint_values[6]]
        jtpt.velocities = [1, 1, 1, 1, 1, 1, 1]
        jtpt.accelerations = [1, 1, 1, 1, 1, 1, 1]
        jtpt.time_from_start = rospy.Duration.from_sec(duration)
        jt_ur10.points.append(jtpt)
        self.joint_publisher.publish(jt_ur10)

    def activate_gripper(self):
        """
        Activate a robot's gripper to grasp objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(True)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def deactivate_gripper(self):
        """
        Deactivate a robot's gripper to release objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(False)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def is_object_attached(self):
        """
        Check whether an object is attached to the gripper

        Returns:
            bool: True if an object is attached, otherwise false
        """
        status = rospy.wait_for_message(
            '/ariac/kitting/arm/gripper/state', VacuumGripperState)
        return status.attached

    def move_arm_base(self, x):
        """
        Only move the joint linear_arm_actuator_joint to the x coordinate

        Args:
            x (float): x position in the world frame
        """
        x = -1.5 - x
        arm_joints = [x, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self._arm_group.go(arm_joints, wait=True)

    def pickandplace(self):
        """
        Pick and Place poses for both parts retrieved dynamically
        """
        rospy.loginfo("Pick A location {}".format(self.pick_coordinates_a))
        rospy.loginfo("Place A location {}".format(self.place_a))
        # Pick and Place part 1
        self.count += 1  # count = 1
        pickup_pose = Pose()
        pickup_pose.position.x = self.pick_coordinates_a[0]
        pickup_pose.position.y = self.pick_coordinates_a[1]
        pickup_pose.position.z = self.pick_coordinates_a[2]

        place_pose = Pose()
        place_pose.position.x = self.place_a[0]
        place_pose.position.y = self.place_a[1]
        place_pose.position.z = self.place_a[2]

        self.move_part(pickup_pose, place_pose)
        rospy.sleep(0.1)

        rospy.loginfo("Final list is {}".format(
            self.final))
        j = 0
        while j < 3:
            self.pick_coordinates_b = self.get_transform(
                "/world", str(self.final[1]))
            j += 1

        self.place_b = self.broadcast_marker(
            self.part_b_info['position_x'], self.part_b_info['position_y'],  self.part_b_info['position_z'], 0, 0, 0, 1, "part_b", str(self.bin_of_parts_to_place[1]))

        rospy.loginfo("Pick B location {}".format(self.pick_coordinates_b))
        rospy.loginfo("Place B location {}".format(
            self.place_b))

        if self.pick_coordinates_b and self.place_b:
            # Pick and Place part 2
            self.count += 1  # count = 2
            pickup_pose = Pose()
            pickup_pose.position.x = self.pick_coordinates_b[0]
            pickup_pose.position.y = self.pick_coordinates_b[1]
            pickup_pose.position.z = self.pick_coordinates_b[2]

            # rospy.sleep(2)
            place_pose = Pose()
            place_pose.position.x = self.place_b[0]
            place_pose.position.y = self.place_b[1]
            place_pose.position.z = self.place_b[2]

            self.move_part(pickup_pose, place_pose)

        if self.count == 2:
            rospy.loginfo("Pick and Place Operation Completed..!")
        else:
            rospy.logerr("Pick and Place not completed...try running again..!")

    def test_arm_base(self):
        """
        Testing the arm base moves to the correct world x
        """
        self.move_arm_base(0)
        rospy.sleep(3.0)
        self.move_arm_base(-1)
        rospy.sleep(3.0)
        self.move_arm_base(-2)
        rospy.sleep(3.0)
        self.move_arm_base(-3)

    def pick_up_part(self, pickup_pose):
        """
        Pick up a part given its pose
        Args:
            pickup_pose (geometry_msgs.Pose): Pose of the part in the 
            world frame
        """

        # First: get the arm closer to the part
        self.move_arm_base(pickup_pose.position.x)

        # This configuration keeps the gripper flat (facing down)
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # position to reach = position of the part
        gripper_position = Pose().position
        gripper_position.x = pickup_pose.position.x
        gripper_position.y = pickup_pose.position.y
        gripper_position.z = pickup_pose.position.z + 0.10

        # combine position + orientation
        above_part_pose = Pose()
        above_part_pose.position = gripper_position
        above_part_pose.orientation = flat_gripper

        # send the gripper to the pose using moveit
        self._arm_group.set_pose_target(above_part_pose)
        self._arm_group.go()

        # activate gripper
        self.activate_gripper()
        # slowly move down until the part is attached to the gripper
        part_is_attached = self.is_object_attached()
        while not part_is_attached:
            pickup_pose = copy.deepcopy(self._arm_group.get_current_pose())
            pickup_pose.pose.position.z -= 0.001
            pickup_pose.pose.orientation = flat_gripper
            self._arm_group.set_pose_target(pickup_pose)
            self._arm_group.go()
            part_is_attached = self.is_object_attached()
            # rospy.sleep(0.2)

        # once the part is attached, lift the gripper
        self._arm_group.set_pose_target(above_part_pose)
        self._arm_group.go()

    def pick_up_part_c(self, pickup_pose):
        """
        Pick up a part given its pose

        Args:
            pickup_pose (geometry_msgs.Pose): Pose of the part in the
            world frame
        """

        # First: get the arm closer to the part
        self.move_arm_base(pickup_pose.position.x)
        # This configuration keeps the gripper flat (facing down)
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # position to reach = position of the part
        gripper_position = Pose().position
        gripper_position.x = pickup_pose.position.x
        gripper_position.y = pickup_pose.position.y
        gripper_position.z = pickup_pose.position.z + 0.10

        # above part pose using cartesian
        above_part_pose_cart_waypoints = []
        above_part_pose_cart = self._arm_group.get_current_pose().pose
        above_part_pose_cart.position = gripper_position
        above_part_pose_cart.orientation = flat_gripper
        above_part_pose_cart_waypoints.append(
            copy.deepcopy(above_part_pose_cart))

        # send the gripper to the pose using moveit(using cartesian path)
        waypoints = []
        wpose = self._arm_group.get_current_pose().pose
        wpose.position = gripper_position
        wpose.orientation = flat_gripper
        waypoints.append(copy.deepcopy(wpose))
        self.cartesian_move(waypoints)

        # activate gripper
        self.activate_gripper()
        # slowly move down until the part is attached to the gripper
        part_is_attached = self.is_object_attached()
        while not part_is_attached:
            pickup_waypoints = []
            wpose = self._arm_group.get_current_pose().pose
            wpose.position.z -= 0.001
            pickup_waypoints.append(copy.deepcopy(wpose))
            self.cartesian_move(pickup_waypoints)
            part_is_attached = self.is_object_attached()

        # once the part is attached, lift the gripper
        self.cartesian_move(above_part_pose_cart_waypoints)

    def place_part_c(self, place_pose):
        """
        Place a part to the given pose

        Args:
            place_pose (geometry_msgs.Pose): Pose of the part in the
            world frame
        """

        # move the arm closer to the drop pose
        self.move_arm_base(place_pose.position.x)

        # ensure the gripper is facing down
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # set the position to reach
        gripper_position = Pose().position
        gripper_position.x = place_pose.position.x
        gripper_position.y = place_pose.position.y
        gripper_position.z = place_pose.position.z + 0.20

        # above part pose using cartesian
        above_part_pose_cart_waypoints = []
        above_part_pose_cart = self._arm_group.get_current_pose().pose
        above_part_pose_cart.position = gripper_position
        above_part_pose_cart.orientation = flat_gripper
        above_part_pose_cart_waypoints.append(
            copy.deepcopy(above_part_pose_cart))

        # send the gripper to the pose using moveit(using cartesian path)
        waypoints = []
        wpose = self._arm_group.get_current_pose().pose
        wpose.position = gripper_position
        wpose.orientation = flat_gripper
        waypoints.append(copy.deepcopy(wpose))
        self.cartesian_move(waypoints)

        # get the pose of the gripper and make it move a bit lower
        # before releasing the part
        place_waypoints = []
        wpose = self._arm_group.get_current_pose().pose
        wpose.position.z -= 0.02
        place_waypoints.append(copy.deepcopy(wpose))
        self.cartesian_move(place_waypoints)
        # deactivate gripper
        self.deactivate_gripper()

        self.cartesian_move(above_part_pose_cart_waypoints)
        # go home
        self.go_home()

    def place_part(self, place_pose):
        """
        Place a part to the given pose

        Args:
            place_pose (geometry_msgs.Pose): Pose of the part in the
            world frame
        """

        # move the arm closer to the drop pose
        self.move_arm_base(place_pose.position.x)

        # ensure the gripper is facing down
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # set the position to reach
        gripper_position = Pose().position
        gripper_position.x = place_pose.position.x
        gripper_position.y = place_pose.position.y
        gripper_position.z = place_pose.position.z + 0.20

        # set the pose = position + orientation
        above_bin_pose = Pose()
        above_bin_pose.position = gripper_position
        above_bin_pose.orientation = flat_gripper
        self._arm_group.set_pose_target(above_bin_pose)
        self._arm_group.go()

        # get the pose of the gripper and make it move a bit lower
        # before releasing the part
        current_arm_pose = copy.deepcopy(self._arm_group.get_current_pose())
        current_arm_pose.pose.position.z -= 0.02
        self._arm_group.set_pose_target(current_arm_pose)
        self._arm_group.go()

        # deactivate gripper
        self.deactivate_gripper()

        # move the arm up
        arm_joints = [current_arm_pose.pose.position.x,
                      0, -1.25, 1.74, -2.66, -1.51, 0]
        self._arm_group.go(arm_joints, wait=True)

        # go home
        self.go_home()

    def move_part(self, pickup_pose, place_pose):
        """
        Move a part from one pose to another pose

        Args:
            pickup_pose (geometry_msgs.Pose): Current pose of the part in world frame
            place_pose (geometry_msgs.Pose): Pose of the part in the bin in the world frame

        Returns:
            bool: True
        """
        # self.pick_up_part(pickup_pose)
        self.pick_up_part_c(pickup_pose)
        self.place_part_c(place_pose)

        return True

    def cartesian_move(self, waypoints):
        """
        Move the robotic arm through waypoints

        Args:
            waypoints (List(geometry_msgs.Pose)): List of waypoints
        """
        self._arm_group.set_pose_reference_frame("world")
        (plan, fraction) = self._arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        self._arm_group.execute(plan, wait=True)

    def go_home(self):
        """
        Move the robotic arm to the 'home' preset
        location
        """
        self.goto_preset_location('home')

    def goto_preset_location(self, location_name):
        """
        Move the robotic arm to a pre-set location

        Args:
            location_name (str): Pre-set location
        """
        arm = self.locations[location_name]
        location_pose = self._arm_group.get_current_joint_values()
        location_pose[:] = arm
        self._arm_group.go(location_pose, wait=True)
