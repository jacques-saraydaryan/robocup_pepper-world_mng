#!/usr/bin/env python
# coding: utf8

import json
import os
import fnmatch
import networkx as nx
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from datetime import datetime  # timedelta
from scipy.spatial import distance

import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.menu_handler import MenuHandler
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.srv import GetMap, GetPlan

from map_manager.srv import *
from robocup_msgs.msg import Graph, Node, Edge

COLORS = {"RED": ColorRGBA(1.0, 0.0, 0.0, 1.0),
          "GREEN": ColorRGBA(0.0, 1.0, 0.0, 1.0),
          "BLUE": ColorRGBA(0.0, 0.0, 1.0, 1.0)
          }
MAP_FRAME = "map"
MAP_RESOLUTION = 0.0    # Resolution of the map in meters/cell
TRUE_VALUE = ["True", "true", "TRUE", 1]
FALSE_VALUE = ["False", "false", "FALSE", 0]


class MapMng:
    """
    Class for the management of the map of the environment of the robot, it makes the link between the objects of the
    other classes (MapNode, MapGraph, MapEdge)
    """

    def __init__(self, _cfgpath):
        """
        Initializes the servers for the management of interactive markers and edges, the subscriber to listen to the
        Clicked Point tool of RViz, an observer and an event handler to watch the modification of the files and a graph of the interactive markers and edges
        """
        self.CONFIG_PATH = _cfgpath  # Path where data files will be saved and loaded
        rospy.loginfo("Path : {}".format(self.CONFIG_PATH))
        self.nodes_list = []  # List of all MapNode objects
        self.edges_list = []  # List of all MapEdge objects
        self.first_node_selected = None  # Variable that keeps track of the first MapNode object used to create an edge
        self.current_map = None

        # Observer and event handler to watch any modifications of the data files
        self.event_handler = MyHandler(self)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, path=self.CONFIG_PATH, recursive=False)
        self.observer.start()

        # Initialization of objects and variables
        MapNode.init_map_mng(self)
        MapEdge.init_map_mng(self)
        self.map_graph = MapGraph()
        self.node_server = InteractiveMarkerServer("pepper_nodes_server")
        self.edge_server = InteractiveMarkerServer("pepper_edges_server")
        self.subscriber = rospy.Subscriber("clicked_point", PointStamped, self.add_node)

        try:
            rospy.wait_for_service("/static_map", 5)
            self.get_map_service_proxy = rospy.ServiceProxy("/static_map", GetMap)
            self.init_map()
        except Exception as e:
            rospy.logwarn("Service static_map call failed : {}".format(e))

        try:
            rospy.wait_for_service("/move_base/make_plan", 5)
            self.path_service_proxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        except Exception as e:
            rospy.logwarn("Service move_base/make_plan call failed : {}".format(e))

        # ROS Services
        self.add_node_service = rospy.Service("/pepper/add_node", AddNode, self.add_node_srv)
        self.get_node_service = rospy.Service("/pepper/get_node", GetNode, self.get_node_srv)
        self.modify_node_service = rospy.Service("/pepper/modify_node", ModifyNode, self.modify_node_srv)
        self.add_edge_service = rospy.Service("/pepper/add_edge", AddEdge, self.add_edge_srv)
        self.send_graph_service = rospy.Service("/pepper/send_graph", SendGraph, self.send_graph_srv)
        self.make_path_service = rospy.Service("/pepper/make_path", MakePath, self.make_path_srv)
        self.update_graph_service = rospy.Service("/pepper/update_graph", UpdateGraph, self.change_node_weight_srv)

    def init_map(self):
        """
        Get the nav_msgs/OccupancyGrid map currently used by the robot
        """
        global MAP_FRAME, MAP_RESOLUTION

        self.current_map = self.get_map_service_proxy().map
        MAP_FRAME = self.current_map.header.frame_id
        MAP_RESOLUTION = self.current_map.info.resolution
        rospy.loginfo("Map frame : {}".format(MAP_FRAME))

    def get_node(self, node_name):
        """
        Return the MapNode object corresponding to the `node_name`
        :param node_name: Name of the MapNode object to find in the list
        :return: The MapNode object if it exists, None otherwise
        """
        nodes = [node for node in self.nodes_list if node.int_marker.name == node_name]
        if len(nodes) < 1:
            rospy.logerr("MapNode object of name {} not found".format(node_name))
            return None
        elif len(nodes) > 1:
            rospy.logerr("Multiple MapNode objects of name {} found, only 1 is allowed".format(node_name))
            return None
        return nodes[0]

    def get_edge(self, edge_name=None, node_1_name=None, node_2_name=None):
        """
        Return the MapEdge object corresponding to the edge name or the two nodes
        :param edge_name: Name of the edge
        :param node_1_name: Name of first node
        :param node_2_name: Name of second node
        :return: The MapEdge object if it exists, None otherwise
        """
        if edge_name is not None:
            edges = [edge for edge in self.edges_list if edge.edge_int_marker.name == edge_name]
        elif node_1_name is not None and node_2_name is not None:
            edges = [edge for edge in self.edges_list if (edge.node_1_name == node_1_name and edge.node_2_name == node_2_name)
                                                      or (edge.node_2_name == node_1_name and edge.node_1_name == node_2_name)]
        else:
            return None
        if len(edges) < 1:
            rospy.logerr("MapEdge object of name {} not found".format(edge_name))
            return None
        elif len(edges) > 1:
            rospy.logerr("Multiple MapEdge objects of name {} found, only 1 is allowed".format(edge_name))
            return None
        return edges[0]

    @staticmethod
    def get_key_from_value(dictionary, value):
        """
        Get the key corresponding to `value` in `dictionary`, only possible in dictionaries where all values are different
        :param dictionary: the dictionary to search
        :param value: the value
        :return:
        """
        for _key in dictionary.keys():
            if dictionary[_key] == value:
                return _key
        rospy.logwarn("No key has a value of {} in dictionary {}".format(value, dictionary))
        return None

    @staticmethod
    def convert_to_bool(string):
        if string in TRUE_VALUE:
            return True
        elif string in FALSE_VALUE:
            return False
        else:
            print("Error, string {} neither in values accepted as True nor as False, returning False as default".format(string))
            return False

    # ----------------
    # Add/Load methods
    def load(self):
        """
        Method that loads all the MapNodes and MapEdges from the CONFIG_PATH folder
        """
        dirs = os.listdir(self.CONFIG_PATH)
        edges_files = []

        for filename in dirs:
            with open(self.CONFIG_PATH + filename, 'r') as f:
                data = eval(json.dumps(json.load(f)))
                if data['type'] == "MapNode":
                    self.add_node(data, True)
                elif data['type'] == "MapEdge":
                    edges_files.append(filename)
                else:
                    rospy.logwarn("Data type not recognized")

        for filename in edges_files:
            with open(self.CONFIG_PATH + filename, 'r') as f:
                data = eval(json.dumps(json.load(f)))
            if not self.add_edge(data['first node'], data['second node'], data['name'], data['weight'], self.convert_to_bool(data['is_crossing_door'])):
                rospy.logwarn("MapNodes of the Edge not found, deleting Edge file")
                os.remove(self.CONFIG_PATH + filename)

        rospy.loginfo(
            "{} MapNodes and {} MapEdges have been loaded".format(len(self.nodes_list), len(self.edges_list)))

    def add_node(self, data, loaded_data=False):
        """
        Method called to create a MapNode object or to load an existing one
        :param data: The data of the MapNode object
        :param loaded_data: True if it's a loaded MapNode so data is a dictionnary, or False so data is a Clicked Point message
        """
        map_node = MapNode(data, loaded_data)

        self.node_server.insert(map_node.int_marker, self.int_mark_process_feedback)
        map_node.menu_handler.apply(self.node_server, map_node.int_marker.name)
        self.node_server.applyChanges()

        self.nodes_list.append(map_node)
        self.map_graph.add_node_to_graph(map_node.int_marker.name)
        if not loaded_data:
            self.save_file(map_node, "MapNode")
        return map_node

    def add_edge(self, node_1_name, node_2_name, name_=None, weight_=1, is_crossing_door_=False):
        """
        Create the MapEdge object
        :param node_1_name: Name of first node
        :param node_2_name: Name of second node
        :param name_: Name of the edge
        :param weight_: Weight for the dijkstra algorithm
        :param is_crossing_door_: True if the edge should represent a door, False otherwise
        :return:
        """
        temp_node_1 = self.get_node(node_1_name)
        temp_node_2 = self.get_node(node_2_name)
        if temp_node_1 is None or temp_node_2 is None:
            rospy.logerr("Edge : First MapNode={} , Second MapNode={}".format(temp_node_1, temp_node_2))
            return False
        if temp_node_1 == temp_node_2:
            rospy.logwarn("Same interactive marker selected twice, select an other couple of nodes")
            return False
        if self.map_graph.is_neighbor(node_1_name, node_2_name):
            rospy.logwarn("Edge already exists, select an other couple of nodes")
            return False

        map_edge = MapEdge(temp_node_1, temp_node_2, name_, weight_, is_crossing_door_)
        temp_node_1.list_of_edges.append(map_edge)
        temp_node_2.list_of_edges.append(map_edge)

        self.edge_server.insert(map_edge.edge_int_marker)
        map_edge.menu_handler.apply(self.edge_server, map_edge.edge_int_marker.name)
        self.edge_server.applyChanges()

        self.edges_list.append(map_edge)
        self.map_graph.add_edge_to_graph(node_1_name, node_2_name, weight_)
        if name_ is None:
            self.save_file(map_edge, "MapEdge")

        return True

    def change_door_mode(self, feedback):
        """
        Defines if the edge in the feedback is crossing a door or not depending on if the box in the menu of the edge is checked or not
        :param feedback: InteractiveMarkerFeedback
        """
        edge = self.get_edge(edge_name=feedback.marker_name)
        handle = feedback.menu_entry_id
        state = edge.menu_handler.getCheckState(handle)

        if state == MenuHandler.CHECKED:
            edge.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            edge.is_crossing_door = False
        else:
            edge.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            edge.is_crossing_door = True

        edge.menu_handler.reApply(self.edge_server)
        self.edge_server.applyChanges()
        self.save_file(edge, "MapEdge")


    # --------------------
    # Delete methods
    def delete_node(self, node_):
        """
        Delete the MapNode
        :param node_: MapNode object
        """
        self.delete_all_edges(node_)
        self.map_graph.delete_node_on_graph(node_.int_marker.name)

        self.node_server.erase(node_.int_marker.name)
        self.node_server.applyChanges()
        self.nodes_list.remove(node_)
        if os.path.exists(self.CONFIG_PATH + node_.int_marker.name + '.json'):
            os.remove(self.CONFIG_PATH + node_.int_marker.name + '.json')

    def delete_all_edges(self, node_):
        """
        Delete all the edges associated to the MapNode object
        :param node_: MapNode object
        """
        for edge_ in reversed(node_.list_of_edges):
            self.delete_edge(edge_)

    def delete_edge(self, edge_):
        """
        Delete the MapEdge object
        :param edge_: MapEdge object
        """
        self.map_graph.delete_edge_on_graph(edge_.node_1_name, edge_.node_2_name)
        node_1 = self.get_node(edge_.node_1_name)
        node_1.list_of_edges.remove(edge_)
        node_2 = self.get_node(edge_.node_2_name)
        node_2.list_of_edges.remove(edge_)

        self.edge_server.erase(edge_.edge_int_marker.name)
        self.edge_server.applyChanges()
        self.edges_list.remove(edge_)
        temp_path = self.CONFIG_PATH + edge_.edge_int_marker.name + '.json'
        if os.path.exists(temp_path):
            os.remove(temp_path)

    # --------------------
    # ROS Services and specific methods
    def add_node_srv(self, req):
        """
        A ROS Service that uses the request `req` to create a MapNode
        :param req: the request containing all the attributes
        :return:
        """
        data = {"name": req.new_node.name,
                "description": req.new_node.name,
                "color": "GREEN",
                "type": "MapNode",
                "pose": {
                    "position": {
                        "x": req.new_node.pose.position.x,
                        "y": req.new_node.pose.position.y,
                        "z": req.new_node.pose.position.z
                    },
                    "orientation": {
                        "x": req.new_node.pose.orientation.x,
                        "y": req.new_node.pose.orientation.y,
                        "z": req.new_node.pose.orientation.z,
                        "w": req.new_node.pose.orientation.w
                    }
                }
                }
        self.save_file(self.add_node(data, True), "MapNode")
        return AddNodeResponse(True)

    def get_node_srv(self, req):
        """
        A ROS Service that returns the name and pose of the MapNode object corresponding to the name in the request
        :param req: The request from the client
        :return: A robocup_msgs/Node message
        """
        node_ = self.get_node(req.name)
        if node_ is None:
            return GetNodeResponse(Node(name="", pose=None))
        return GetNodeResponse(Node(name=node_.int_marker.name, pose=node_.int_marker.pose))

    def modify_node_srv(self, req):
        """
        A ROS Service to modify some attributes of a MapNode
        :param req: The request with the attribute to change and it's new value
        :return: True if it succeeded changing the attribute, False otherwise
        """
        node_ = self.get_node(req.node_name)
        if node_ is None:
            rospy.logwarn("The Node doesn't exist, check the name you entered in the request")
            return ModifyNodeResponse(False)

        if req.attribute == "color" and req.new_value in COLORS:
            for _control in node_.int_marker.controls:
                if _control.name == "marker":
                    _control.markers[0].color = COLORS[req.new_value]
        else:
            rospy.logwarn("Attribute can't be modified or the new value is not allowed")
            return ModifyNodeResponse(False)

        self.node_server.erase(node_.int_marker.name)
        self.node_server.applyChanges()
        self.node_server.insert(node_.int_marker, self.int_mark_process_feedback)
        node_.menu_handler.apply(self.node_server, node_.int_marker.name)
        self.node_server.applyChanges()

        self.save_file(node_, "MapNode")
        return ModifyNodeResponse(True)

    def add_edge_srv(self, req):
        """
        A ROS Service that adds an edge between the two MapNodes in the request
        :param req: Contains the name of the two MapNodes, its weight and a boolean if it crosses a door or not
        :return:
        """
        success = self.add_edge(req.first_node, req.second_node, None, req.weight, req.is_crossing_door)
        return AddEdgeResponse(success)

    def send_graph_srv(self, req):
        """
        A ROS Service to return an object of type Graph from the map_manager messages, containing ItM and Edge messages.
        :param req: The request is empty
        :return: The Graph object
        """
        graph_resp = Graph()
        for node_ in self.nodes_list:
            graph_resp.nodes.append(Node(name=node_.int_marker.name, pose=node_.int_marker.pose))
        for edge_ in self.edges_list:
            graph_resp.edges.append(Edge(first_node=edge_.node_1_name, second_node=edge_.node_2_name,
                                         weight=edge_.weight, is_crossing_door=edge_.is_crossing_door))
        return SendGraphResponse(graph_resp)

    def change_node_weight_srv(self, req):
        """
        a service to change the weight of edges associated with the node in the request
        :param req: Request of client
        :return:
        """
        if req.act == "reset":
            for edge in self.edges_list:
                edge.edge_weight = 1.0
                self.save_file(edge, "MapEdge")
                self.map_graph.change_edge_weight(edge.node_1_name, edge.node_2_name, 1.0)
        else:
            node_ = self.get_node(req.node)
            for edge in node_.list_of_edges:
                edge.edge_weight = req.weight
                self.save_file(edge, "MapEdge")
                self.map_graph.change_edge_weight(edge.node_1_name, edge.node_2_name, req.weight)

        return UpdateGraphResponse(True)

    def make_path_srv(self, req):
        """
        A ROS Service to return the list of interactive markers to go from one interactive marker to another.
        It uses the Dijkstra algorithm from the Networkx package.
        :param req: The request containing the two points in space
        :return: The list of the interactive markers between the start and the end and the list of edges
        """
        path = Graph()

        if req.mode == "fast_ros_plan":
            ros_make_plan = self.path_service_proxy(req.start, req.end, 0.0)
            result = self.check_nodes_on_path(ros_make_plan.plan.poses)
            start_node = result[0]
            end_node = result[-1]
        else:
            if req.mode != "euclidian" and req.mode != "ros_plan":
                req.mode = "euclidian"
            res1 = self.find_nearest_node(req.start.pose, req.mode)
            res2 = self.find_nearest_node(req.end.pose, req.mode)
            start_node = res1[0]
            end_node = res2[0]

        tmp_path = self.map_graph.make_path(start_node.int_marker.name, end_node.int_marker.name)
        if tmp_path is None:
            return MakePathResponse(None)

        for i in range(len(tmp_path)):
            map_node = self.get_node(tmp_path[i])
            path.nodes.append(Node(name=map_node.int_marker.name, pose=map_node.int_marker.pose))
            if i < len(tmp_path) - 1:
                map_edge = self.get_edge(node_1_name=tmp_path[i], node_2_name=tmp_path[i+1])
                path.edges.append(Edge(first_node=map_edge.node_1_name, second_node=map_edge.node_2_name,
                                        weight=map_edge.edge_weight, is_crossing_door=map_edge.is_crossing_door))

        rospy.loginfo("Nodes : {}".format(tmp_path))
        rospy.loginfo("Edges : {}".format(path.edges))
        return MakePathResponse(path)

    def find_nearest_node(self, initial_point, mode, min_distance=0):
        """
        Returns the nearest MapNode object of the `initial_point` in the map
        :param mode: "euclidian" to measure the euclidian distance between two points, otherwise it measure the
        distance taking account of obstacles
        :param min_distance: The minimal distance the MapNode object will be accepted
        :param initial_point: A geometry_msgs/Pose
        :return: The MapNode object and its distance from the point
        """
        _list = []

        if mode == "euclidian":
            for _node in self.nodes_list:
                point1 = [initial_point.position.x, initial_point.position.y, initial_point.position.z]
                point2 = [_node.int_marker.pose.position.x, _node.int_marker.pose.position.y,
                          _node.int_marker.pose.position.z]
                dist = distance.euclidean(point1, point2)
                if min_distance < dist:
                    _list.append([_node, dist])
        else:
            ps1 = self.convert_to_pose_stamped(initial_point)
            for _node in self.nodes_list:
                ps2 = self.convert_to_pose_stamped(_node.int_marker.pose)
                dist = self.measure_distance(ps1, ps2)
                if min_distance < dist:
                    _list.append([_node, dist])

        _list.sort(key=lambda _list: _list[1])
        return _list[0]

    def check_nodes_on_path(self, _path):
        """
        Finds MapNodes on the path in parameter
        :param _path: List of points in the map
        :return:
        """
        max_dist_around_point = 2
        nodes_on_path = []

        for i in range(0, len(_path), 20):
            min_dist = float('inf')
            node_close = None

            pt = _path[i]
            x_min = pt.pose.position.x - max_dist_around_point
            x_max = pt.pose.position.x + max_dist_around_point
            y_min = pt.pose.position.y - max_dist_around_point
            y_max = pt.pose.position.y + max_dist_around_point

            for node_ in self.nodes_list:
                x = node_.int_marker.pose.position.x
                y = node_.int_marker.pose.position.y
                if x_min < x < x_max and y_min < y < y_max:
                    pt2 = self.convert_to_pose_stamped(node_.int_marker.pose)
                    dist = self.measure_distance(pt, pt2)
                    if dist < min_dist:
                        node_close = node_
                        min_dist = dist

            if node_close is not None and node_close not in nodes_on_path:
                nodes_on_path.append(node_close)

        return nodes_on_path

    def measure_distance(self, point_1, point_2):
        """
        Return the minimal distance between the two points that avoid obstacles
        :param point_1: A geometry_msgs/PoseStamped
        :param point_2: A geometry_msgs/PoseStamped
        :return: The distance between the two points avoiding obstacles
        """
        _path = self.path_service_proxy(point_1, point_2, 0.0)
        if len(_path.plan.poses) == 0:
            return float("inf")
        elif len(_path.plan.poses) == 1:
            return 0.0

        total = 0.0
        prev_pose = _path.plan.poses[0]
        for cur_pose in _path.plan.poses[1:len(_path.plan.poses)]:
            total = total + np.linalg.norm([cur_pose.pose.position.x - prev_pose.pose.position.x,
                                            cur_pose.pose.position.y - prev_pose.pose.position.y])
            prev_pose = cur_pose

        return total

    @staticmethod
    def convert_to_pose_stamped(pose):
        """
        Convert geometry_msgs/Pose in geometry_msgs/PoseStamped
        :param pose:
        :return:
        """
        tmp = PoseStamped()
        tmp.header.frame_id = MAP_FRAME
        tmp.header.stamp = rospy.Time.now()
        tmp.pose = pose

        return tmp

    # --------------------
    # Save/update and file modification methods
    def int_mark_process_feedback(self, feedback):
        """
        Method called when the InteractiveMarkerServer receives the information that a MapNode is being moved
        :param feedback: InteractiveMarkerFeedback
        """
        tmp_node = self.get_node(feedback.marker_name)
        for edge in tmp_node.list_of_edges:
            edge.update_edge(feedback)
            self.edge_server.erase(edge.edge_int_marker)
            self.edge_server.insert(edge.edge_int_marker)
            self.edge_server.applyChanges()

        self.node_server.applyChanges()
        self.save_file(tmp_node, "MapNode")

    def save_file(self, obj, type_of_obj):
        """
        Save/update the data file of the MapNode or MapEdge object
        :param obj: The MapNode or MapEdge object that will be saved in a file
        :param type_of_obj: Type of the object parameter, either MapNode or MapEdge
        """
        if type_of_obj == "MapNode":
            with open(self.CONFIG_PATH + obj.int_marker.name + '.json', 'w+') as f:
                pose_dict = {"type": "MapNode",
                             "name": obj.int_marker.name,
                             "description": obj.int_marker.description,
                             "color": self.get_key_from_value(COLORS, obj.marker.color),
                             "pose": {
                                 "position": {
                                     "x": obj.int_marker.pose.position.x,
                                     "y": obj.int_marker.pose.position.y,
                                     "z": obj.int_marker.pose.position.z
                                 },
                                 "orientation": {
                                     "x": obj.int_marker.pose.orientation.x,
                                     "y": obj.int_marker.pose.orientation.y,
                                     "z": obj.int_marker.pose.orientation.z,
                                     "w": obj.int_marker.pose.orientation.w
                                 }
                             }
                             }
                f.write(json.dumps(pose_dict, indent=4, sort_keys=True))

        elif type_of_obj == "MapEdge":
            with open(self.CONFIG_PATH + obj.edge_int_marker.name + '.json', 'w+') as f:
                pose_dict = {"type": "MapEdge",
                             "name": obj.edge_int_marker.name,
                             "first node": obj.node_1_name,
                             "second node": obj.node_2_name,
                             "weight": obj.edge_weight,
                             "is_crossing_door": str(obj.is_crossing_door)
                             }
                f.write(json.dumps(pose_dict, indent=4))

        else:
            rospy.logwarn("Type of object to save in a file not recognized")

    def created_file(self, filepath):
        """
        Method called when a file is created to check if it's a MapNode or MapEdge data file. Then it checks if the object
        of the corresponding type exists and if not it calls the method to create it
        :param filepath: Path of the file created
        """
        with open(filepath, 'r') as f:
            data = eval(json.dumps(json.load(f)))
            if data['type'] == "MapNode":
                if self.get_node(data['name']) is None:
                    self.add_node(data, True)
            elif data['type'] == "MapEdge":
                if self.get_edge(edge_name=data['name']) is None:
                    self.add_edge(data['first node'], data['second node'], data['name'], data['weight'], self.convert_to_bool(data['is_crossing_door']))
            else:
                rospy.logwarn("Data type not recognized")

    def modified_file(self, filepath):
        """
        Method called when a file is modified to check if it's a MapNode or MapEdge data file. Then it retrieves the corresponding
        object in the list of the map manager and checks if it needs an update on the map
        :param filepath: Path of the file modified
        """
        with open(filepath, 'r') as f:
            data = eval(json.dumps(json.load(f)))
            if data['type'] == "MapNode":
                temp_node = self.get_node(data['name'])
                if not self.compare_data(temp_node.int_marker.description, data['description']):
                    temp_node.int_marker.description = data['description']
                if not self.compare_data(temp_node.int_marker.pose.position.x, data['pose']['position']['x']):
                    temp_node.int_marker.pose.position.x = data['pose']['position']['x']
                if not self.compare_data(temp_node.int_marker.pose.position.y, data['pose']['position']['y']):
                    temp_node.int_marker.pose.position.y = data['pose']['position']['y']
                if not self.compare_data(temp_node.int_marker.pose.position.z, data['pose']['position']['z']):
                    temp_node.int_marker.pose.position.z = data['pose']['position']['z']
                if not self.compare_data(temp_node.int_marker.pose.orientation.x, data['pose']['orientation']['x']):
                    temp_node.int_marker.pose.orientation.x = data['pose']['orientation']['x']
                if not self.compare_data(temp_node.int_marker.pose.orientation.y, data['pose']['orientation']['y']):
                    temp_node.int_marker.pose.orientation.y = data['pose']['orientation']['y']
                if not self.compare_data(temp_node.int_marker.pose.orientation.z, data['pose']['orientation']['z']):
                    temp_node.int_marker.pose.orientation.z = data['pose']['orientation']['z']
                if not self.compare_data(temp_node.int_marker.pose.orientation.w, data['pose']['orientation']['w']):
                    temp_node.int_marker.pose.orientation.w = data['pose']['orientation']['w']
                self.node_server.applyChanges()
            elif data['type'] == "MapEdge":
                pass
            else:
                rospy.logwarn("Data type not recognized")

    @staticmethod
    def compare_data(data_1, data_2):
        """
        Method that compares `data_1` and `data_2`
        :param data_1: First data to compare
        :param data_2: Second data to compare
        :return: True if both data are equal, False otherwise
        """
        if data_1 != data_2:
            return False
        return True

    def deleted_file(self):
        """
        Method called when a file is deleted to check if it's a MapNode or MapEdge data file. Then it retrieves the corresponding
        object in the list of the map manager and deletes it
        """
        for node_ in self.nodes_list:
            if not os.path.exists(self.CONFIG_PATH + node_.int_marker.name + ".json"):
                rospy.loginfo("File of node {} does not exist, deleting it...".format(node_.int_marker.name))
                self.delete_node(node_)
        for edge_ in self.edges_list:
            if not os.path.exists(self.CONFIG_PATH + edge_.edge_int_marker.name + ".json"):
                rospy.loginfo("File of edge {} does not exist, deleting it...".format(edge_.edge_int_marker.name))
                self.delete_edge(edge_)


class MyHandler(FileSystemEventHandler):
    """
    Class for the creation of an event handler that watches the data files and tells the Map manager if modifications
    are made
    """

    def __init__(self, _map_mng):
        """
        Initialization of the event handler attributes, the map_mng is used in the on_any_event method to call MapMng
        methods when there is a modification
        :param _map_mng: The manager created at the beginning of the program
        """
        self.map_mng = _map_mng
        self.last_modified = datetime.now()

    def on_created(self, event):
        """
        Method called when a file is created in the CONFIG_PATH folder
        :param event: Event that happens to a file
        """
        rospy.loginfo('Event type: {} file {}'.format(event.event_type, os.path.basename(event.src_path)))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.created_file(event.src_path)

    def on_modified(self, event):
        """
        Method called when a file is modified in the CONFIG_PATH folder
        :param event: Event that happens to a file
        """
        """
        if datetime.now() - self.last_modified < timedelta(seconds=0.1):
            return
        self.last_modified = datetime.now()
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.modified_file(event.src_path)
        """
        pass

    def on_deleted(self, event):
        """
        Method called when a file is deleted in the CONFIG_PATH folder
        :param event: Event that happens to a file
        """
        rospy.loginfo('Event type: {} file {}'.format(event.event_type, os.path.basename(event.src_path)))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.deleted_file()


class MapGraph:
    """
    Class for the creation of a Graph object, which contains a Networkx graph that draws all the Nodes and Edges
    """

    def __init__(self):
        """
        Initialize the Networkx Graph object
        """
        self.graph = nx.Graph()

    def add_node_to_graph(self, _node):
        """
        Add a node `_node` to the graph
        :param _node: Name of the node to be added (usually it's the name of the interactive marker)
        """
        self.graph.add_node(_node)

    def add_edge_to_graph(self, first_node, second_node, _weight=1):
        """
        Add an edge between `first_node` and `second_node` to the graph
        :param _weight: weight of the node
        :param first_node: Name of the first node of the edge
        :param second_node: Name of the second node of the edge
        """
        self.graph.add_edge(first_node, second_node, weight=_weight)

    def delete_node_on_graph(self, _node):
        """
        Delete the node `_node` from the graph
        :param _node: Name of the node to be removed
        """
        if self.graph.has_node(_node):
            self.graph.remove_node(_node)

    def delete_edge_on_graph(self, first_node, second_node):
        """
        Remove the edge of the graph connecting `first_node` and `second_node`
        :param first_node: Name of the first node of the edge
        :param second_node: Name of the second node of the edge
        """
        if self.is_neighbor(first_node, second_node):
            self.graph.remove_edge(first_node, second_node)

    def is_neighbor(self, node_1, node_2):
        """
        A method to test if `node_1` and `node_2` are connected by an edge
        :param node_1: First node of the graph
        :param node_2: Second node of the graph
        :return: True if they are connected, False otherwise
        """
        for n in self.graph.neighbors(node_1):
            if node_2 == n:
                return True
        return False

    def make_path(self, start_node, end_node):
        """
        Method that returns the list of nodes to go from `start_node` to `end_node`.
        It uses the Dijkstra algorithm from the Networkx package.
        :param start_node: The starting node of the path
        :param end_node: The ending node of the path
        :return: The list of the nodes between the start and the end if there's no errors
        """
        try:
            path = nx.dijkstra_path(self.graph, start_node, end_node)
            return path
        except nx.NodeNotFound or nx.NetworkXNoPath:
            rospy.logwarn("Node or path not found")
            return None

    def change_edge_weight(self, _node1, _node2, _weight):
        """
        Changes the weight of all edges of a node to the given value in parameter
        :param _weight: Value to give to edges
        :param _node1: Name of the first node of the edge
        :param _node2: Name of the second node
        :return:
        """
        if self.graph.has_edge(_node1, _node2):
            self.graph.edges[_node1, _node2]['weight'] = _weight
            rospy.logwarn("Weight of edge {}-{} set to {}".format(_node1, _node2, self.graph[_node1][_node2]['weight']))


class MapNode:
    """
    Class to create a MapNode object containing an Interactive Marker from the visualization_msgs package and some
    other attributes and methods
    """
    current_id = 0  # ID of markers that increments each time a MapNode object is created, so they're unique
    map_mng = None

    def __init__(self, data, loaded_data=False):
        """
        Initialization of a new MapNode object
        """
        self.int_marker = InteractiveMarker()
        self.marker = Marker()
        self.menu_handler = MenuHandler()
        self.list_of_edges = []  # List of all Edge objects associated with the MapNode object

        self.make_int_marker(data, loaded_data)

    @staticmethod
    def init_map_mng(map_mng_):
        MapNode.map_mng = map_mng_

    def make_int_marker(self, data, loaded_data=False):
        """
        Create the Interactive Marker, its controls and saves it in a file if it is a new MapNode object
        :param data: The data used to create the Interactive Marker, either a Point Stamped message or a dictionnary
        containing the position of the marker and its other attributes
        :param loaded_data: Tells the method which type of data is received, False if it is a new MapNode object so `data`
        is a Point Stamped message and True if it is a loaded data from the json/ directory so `data` is a dictionnary
        """
        self.int_marker.header.frame_id = MAP_FRAME
        self.int_marker.scale = 0.7

        if not loaded_data:
            self.int_marker.name = "Node_" + str(MapNode.current_id)
            self.int_marker.description = self.int_marker.name
            self.int_marker.header.stamp = data.header.stamp
            self.int_marker.pose.position = data.point
            self.int_marker.pose.position.z = 0.005
            self.int_marker.pose.orientation.w = 1.0
            self.make_marker("GREEN")
            MapNode.current_id += 1
        else:
            self.int_marker.name = data['name']
            self.int_marker.description = data['description']
            self.int_marker.header.stamp = rospy.Time.now()
            self.int_marker.pose.position.x = data['pose']['position']['x']
            self.int_marker.pose.position.y = data['pose']['position']['y']
            self.int_marker.pose.position.z = data['pose']['position']['z']
            self.int_marker.pose.orientation.x = data['pose']['orientation']['x']
            self.int_marker.pose.orientation.y = data['pose']['orientation']['y']
            self.int_marker.pose.orientation.z = data['pose']['orientation']['z']
            self.int_marker.pose.orientation.w = data['pose']['orientation']['w']
            self.make_marker(data['color'])
            if int(data['name'][5:]) >= MapNode.current_id:
                MapNode.current_id = int(data['name'][5:]) + 1
        self.make_controls()
        self.add_menu()

    def make_controls(self):
        """
        Method making all the controls of the Interactive marker so it can be moved around the map and a menu can be open
        by right clicking on the arrow marker
        """
        # interactive control containing the menu and the marker
        control = InteractiveMarkerControl()
        control.name = "marker"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(self.marker)
        self.int_marker.controls.append(control)

        # control for the translation on the x axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(control)

        # control for the translation on the y axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(control)

        # control for the rotation around the z axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(control)

    def make_marker(self, _color):
        """
        Make an arrow marker that represents the interactive marker
        :return: The Marker object
        """
        if _color not in COLORS:
            _color = "GREEN"
        self.marker.type = Marker.ARROW
        self.marker.scale.x = self.int_marker.scale * 0.5
        self.marker.scale.y = self.int_marker.scale * 0.2
        self.marker.scale.z = self.int_marker.scale * 0.2
        self.marker.color = COLORS[_color]

        return self.marker

    def add_menu(self):
        """
        Add a menu to the Interactve marker that allows some actions on it
        """
        self.menu_handler.insert("Create an edge", callback=self.create_edge)
        self.menu_handler.insert("Delete all edges", callback=self.delete_all_edges)
        self.menu_handler.insert("Delete marker", callback=self.delete_self)

    def create_edge(self, feedback):
        if MapNode.map_mng.first_node_selected is None:
            MapNode.map_mng.first_node_selected = self
        else:
            MapNode.map_mng.add_edge(MapNode.map_mng.first_node_selected.int_marker.name, self.int_marker.name)
            MapNode.map_mng.first_node_selected = None

    def delete_all_edges(self, feedback):
        MapNode.map_mng.delete_all_edges(self)

    def delete_self(self, feedback):
        MapNode.map_mng.delete_node(self)


class MapEdge:
    """
    Class to create a MapEdge object which is an Interactive marker between two Interactive marker MapNodes
    """
    current_id = 0  # Id given to the next edge created if it is a new edge, increasing by one so all edges have a different Id
    map_mng = None

    @staticmethod
    def init_map_mng(map_mng_):
        MapEdge.map_mng = map_mng_

    def __init__(self, node_1, node_2, _name=None, _weight=1.0, _is_crossing_door=False):
        """
        Initialization of a new edge created by the two MapNode objects
        :param node_1: The first MapNode of the edge
        :param node_2: The second MapNode of the edge
        :param _name: Name of the edge if it's a loaded edge
        """
        self.edge_int_marker = InteractiveMarker()
        self.edge_marker = Marker()
        self.menu_handler = MenuHandler()
        self.start_point = Point()
        self.end_point = Point()

        # Create the interactive marker, marker, controls and menu
        self.make_int_marker(_name)
        self.make_marker(node_1.int_marker.pose.position, node_2.int_marker.pose.position)
        self.make_control()
        self.make_menu()

        # Some attributes
        self.edge_weight = _weight
        self.is_crossing_door = _is_crossing_door
        self.node_1_name = node_1.int_marker.name
        self.node_2_name = node_2.int_marker.name
        if self.is_crossing_door:
            self.menu_handler.setCheckState(self.door_menu, MenuHandler.CHECKED)

    def make_int_marker(self, _name):
        self.edge_int_marker.header.frame_id = MAP_FRAME
        self.edge_int_marker.header.stamp = rospy.Time.now()
        self.edge_int_marker.scale = 0.1

        if _name is None:
            # New edge
            self.edge_int_marker.name = "Edge_" + str(MapEdge.current_id)
            MapEdge.current_id += 1
        else:
            # It's an edge being loaded
            self.edge_int_marker.name = _name
            if int(_name[5:]) >= MapEdge.current_id:
                # It sets the current_id to the maximum id of the loaded edges
                MapEdge.current_id = int(_name[5:]) + 1

    def make_control(self):
        # interactive control containing the menu and the marker
        control = InteractiveMarkerControl()
        control.name = "marker"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(self.edge_marker)
        self.edge_int_marker.controls.append(control)

    def make_marker(self, first_point, second_point):
        # Marker line
        self.edge_marker.type = Marker.LINE_STRIP
        self.edge_marker.color = COLORS["RED"]
        self.edge_marker.scale.x = self.edge_int_marker.scale

        # Starting point of the line
        self.start_point = first_point
        self.edge_marker.points.append(self.start_point)

        # Ending point of the line
        self.end_point = second_point
        self.edge_marker.points.append(self.end_point)

        return self.edge_marker

    def make_menu(self):
        self.menu_handler.insert("Delete edge", callback=self.delete_edge)
        self.door_menu = self.menu_handler.insert("Door", callback=MapEdge.map_mng.change_door_mode)
        self.menu_handler.setCheckState(self.door_menu, MenuHandler.UNCHECKED)

    def update_edge(self, feedback):
        """
        Update the position of the edge according to the two nodes that defines it.
        This method is called by the processFeedback method of the node
        :param feedback: The node moving on the map
        """
        if feedback.marker_name == self.node_1_name:
            self.edge_marker.points[0] = feedback.pose.position
        elif feedback.marker_name == self.node_2_name:
            self.edge_marker.points[1] = feedback.pose.position
        else:
            rospy.logerr("Error, the node is not associated with the edge")

    def delete_edge(self, feedback):
        MapEdge.map_mng.delete_edge(self)


if __name__ == '__main__':
    rospy.init_node("pepper_map_manager")

    default_value = rospy.get_param("/map_file")
    dir_name = rospy.get_param("~confPath", default_value)
    _cfgpath = os.path.join(os.path.dirname(__file__), '../json/') + os.path.splitext(os.path.basename(dir_name))[0]
    if not os.path.isdir(_cfgpath):
        rospy.logwarn("Directory does not exist, creating it...")
        os.mkdir(_cfgpath)
        rospy.loginfo("Directory created")
    if _cfgpath[-1] != "/":
        _cfgpath = _cfgpath + "/"
    manager = MapMng(_cfgpath)
    manager.load()
    rospy.spin()
