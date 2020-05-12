#!/usr/bin/env python
# coding: utf8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from map_manager.srv import *
from geometry_msgs.msg import Pose
import tf


class TestInt:
    def __init__(self):
        rospy.init_node('test_integration')
        self.current_pose = Pose()
        self.goal = Pose()
        self._tflistener = tf.TransformListener()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goto_point_service = rospy.Service('/pepper/goto_point', GotoPoint, self.simple_nav_srv)

    def get_current_pose(self):
        now = rospy.Time.now()

        self._tflistener.waitForTransform("/map", "/base_link", now, rospy.Duration(2))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]
        robotPose.orientation.x = rot[0]
        robotPose.orientation.y = rot[1]
        robotPose.orientation.z = rot[2]
        robotPose.orientation.w = rot[3]

        return robotPose

    def simple_nav_srv(self, _goal):
        self.current_pose = self.get_current_pose()
        goto_point_service = rospy.ServiceProxy('/pepper/make_path', MakePath)
        path = goto_point_service(self.current_pose, _goal.point)
        for _itm in path.list_of_itms:
            result = self.go_to_pose(_itm.pose)
            if result is None:
                rospy.logwarn("When trying to reach {}, result is None".format(_itm.name))
                return
        self.go_to_pose(_goal.point)
        return GotoPointResponse(True)

    def go_to_pose(self, _pose):

        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position = _pose.position
        self.goal.target_pose.pose.orientation = _pose.orientation

        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    try:
        nav_int_test = TestInt()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
