#!/usr/bin/env python


"""
Unpause gazebo after all models have been spawned.
"""

if __name__ == '__main__':
    import rospy
    from std_srvs.srv import Empty
    import time

    # Copied from https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/scripts/spawn_model
    rospy.logwarn("Waiting for unpause physics topic")
    rospy.wait_for_service('/gazebo/unpause_physics')
    # TODO(sloretz) check if all arm models spawned before unpausing
    rospy.logwarn("HACK sleeping before unpausing model")
    rospy.sleep(10)
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
    except rospy.ServiceException as e:
        rospy.logerr("Unpause physics service call failed: %s", e)
