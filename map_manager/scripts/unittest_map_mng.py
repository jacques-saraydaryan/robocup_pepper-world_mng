#!/usr/bin/env python

import unittest
import rospy
import os

from map_mng import MapMng
from map_manager.srv import *
from geometry_msgs.msg import Pose, Point
from robocup_msgs.msg import Node, Edge


class TestIntMark(unittest.TestCase):

    def setUp(self):
        rospy.init_node("pepper_interactive_marker_test")

    def test_load(self):
        default_value = os.path.join(os.path.dirname(__file__), '../json/')
        self.manager = MapMng(default_value)
        self.manager.load()

    def test_find_near_itm(self):
        self.manager.find_nearest_node([2.0, 0.5, 0.0], "")

    def test_send_graph(self):
        rospy.wait_for_service('/pepper/send_graph')
        graph_handle = rospy.ServiceProxy('/pepper/send_graph', SendGraph)
        self.graph_obj = graph_handle()

    def test_get_itm(self):
        rospy.wait_for_service('/pepper/get_node')
        handle = rospy.ServiceProxy('/pepper/get_node', GetNode)
        self.node = handle("Node_0")
        rospy.spin()

    def test_add_itm_srv(self):
        rospy.wait_for_service('/pepper/add_node')
        handle = rospy.ServiceProxy('/pepper/add_node', AddNode)
        pose1 = Pose()
        pose1.position.x = -5.0
        pose1.position.y = 6.0
        pose1.position.z = 0
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 0.0
        node = Node(name="ItM5", nature="None", pose=pose1)
        self.success = handle(node)
        rospy.spin()

    def test_add_edge_srv(self):

        rospy.wait_for_service('/pepper/add_edge')
        handle = rospy.ServiceProxy('/pepper/add_edge', AddEdge)
        success = handle(Edge(first_node="ItM0", second_node="ItM3", weight=1, is_crossing_door=False))
        rospy.spin()

    def test_obstacles(self):
        default_value = os.path.join(os.path.dirname(__file__), '../json/')
        self.manager = MapMng(default_value)

        ps1 = Point(x=-2.0, y=5.35, z=0.002)
        ps2 = Point(x=-2.38, y=8.19, z=0.002)
        chk_obs = self.manager.measure_distance(ps1, ps2)
        rospy.spin()


if __name__ == '__main__':
    unittest.main()
