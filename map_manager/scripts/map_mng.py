#!/usr/bin/env python
# coding: utf8

import json
import os
import time
import fnmatch
import networkx as nx
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from datetime import datetime, timedelta
from scipy.spatial import distance

import rospy
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.menu_handler import MenuHandler
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.srv import GetMap, GetPlan

from map_manager.srv import *
from robocup_msgs.msg import Graph, ItM, Edge


class MapMng:
    """
    Class for the management of the map of the environment of the robot, it makes the link between the objects of the
    other classes (IntMark abbreviated as ItM, Graph, Edges)
    """
    def __init__(self, _cfgpath):
        """
        Initializes the server for the management of interactive markers, the publisher for the display of edges, the
        subscriber to listen to the Clicked Point tool of RViz, an observer and an event handler to watch the
        modification of the files
        """
        self.CONFIG_PATH = _cfgpath  # Path where data files will be saved and loaded
        rospy.loginfo("Path : {}".format(self.CONFIG_PATH))
        self.itm_list = []  # List of all IntMark objects
        self.edges_list = []  # List of all Edge objects
        self.start_temp_mark = ""  # Variable that keeps track of the first IntMark object used to create an edge
        self.starting_point_created = False  # Variable for the creation of a new edge used in the create_edge method
        self.current_map = None

        # Observer and event handler to watch any modifications of the data files
        self.event_handler = MyHandler(self)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, path=self.CONFIG_PATH, recursive=False)
        self.observer.start()

        # Initialization of objects and variables
        MapItM.init_map_mng(self)
        self.map_graph = MapGraph()
        self.server = InteractiveMarkerServer("pepper_interactive_markers")
        self.edge_pub = rospy.Publisher("interactive_marker_edges", Marker, queue_size=10)
        self.subscriber = rospy.Subscriber("clicked_point", PointStamped, self.add_itm)
        rospy.wait_for_service("/static_map", 5)
        self._getMap = rospy.ServiceProxy("/static_map", GetMap)
        rospy.wait_for_service("/move_base/make_plan", 5)
        self.path = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        # ROS Services
        self.add_itm_service = rospy.Service("/pepper/add_itm", AddItM, self.add_itm_srv)
        self.get_itm_service = rospy.Service("/pepper/get_itm", GetItM, self.get_itm_srv)
        self.add_edge_service = rospy.Service("/pepper/add_edge", AddEdge, self.add_edge_srv)
        self.send_graph_service = rospy.Service("/pepper/send_graph", SendGraph, self.send_graph_srv)
        self.make_path_service = rospy.Service("/pepper/make_path", MakePath, self.make_path_srv)
        self.update_graph_service = rospy.Service("/pepper/update_graph", UpdateGraph, self.change_node_weight_srv)

    def get_itm(self, itm_name):
        """
        Return the IntMark object corresponding to the `itm_name`
        :param itm_name: Name of the IntMark object to find in the list
        :return: The IntMark object if it exists, None otherwise
        """
        itms = [itm for itm in self.itm_list if itm.int_marker.name==itm_name]
        if len(itms) < 1:
            rospy.loginfo("IntMark object of name {} not found".format(itm_name))
            return None
        elif len(itms) > 1:
            rospy.loginfo("Multiple IntMark objects of name {} found, only 1 is allowed".format(itm_name))
            return None
        return itms[0]

    def get_edge(self, edge_id):
        """
        Return the Edge object corresponding to `edge_id`
        :param edge_id: Id of the Edge object to find in the list
        :return: The Edge object if it exists, None otherwise
        """
        edges = [edge for edge in self.edges_list if edge.edge_marker.id==edge_id]
        if len(edges) < 1:
            rospy.loginfo("Edge object of Id {} not found".format(edge_id))
            return None
        elif len(edges) > 1:
            rospy.loginfo("Multiple Edge objects of Id {} found, only 1 is allowed".format(edge_id))
            return None
        return edges[0]

    # ----------------
    # Add/Load methods
    def load(self):
        """
        Method that loads all existing markers and puts them in the Interactive Marker Server and then the edges between
        the markers at the launch of the program
        """
        dirs = os.listdir(self.CONFIG_PATH)
        edges_files = []

        for filename in dirs:
            with open(self.CONFIG_PATH + filename, 'r') as f:
                data = eval(json.dumps(json.load(f)))
                if data['type'] == "IntMark":
                    self.add_itm(data, True)
                elif data['type'] == "Edge":
                    edges_files.append(filename)
                else:
                    rospy.logwarn("Data type not recognized")

        for filename in edges_files:
            with open(self.CONFIG_PATH + filename, 'r') as f:
                data = eval(json.dumps(json.load(f)))
            if not self.load_edge(data):
                rospy.logwarn("IntMark of the Edge not found, deleting Edge file")
                os.remove(self.CONFIG_PATH + filename)

        rospy.loginfo("{} Interactive Markers and {} Edges have been loaded".format(len(self.itm_list), len(self.edges_list)))

    def add_itm(self, data, loaded_data=False):
        """
        Method called to create an IntMark object or to load an existing one
        :param data: The data of the IntMark object
        :param loaded_data: True if it's a loaded IntMark so data is a dictionnary from the .json file, False otherwise
        so data is a Clicked Point message
        """
        new_int_mark = MapItM()
        new_int_mark.make_inter_marker(data, loaded_data)

        self.server.insert(new_int_mark.int_marker, self.int_mark_process_feedback)
        new_int_mark.menu_handler.apply(self.server, new_int_mark.int_marker.name)
        self.server.applyChanges()

        self.itm_list.append(new_int_mark)
        self.map_graph.add_marker_to_graph(new_int_mark.int_marker.name)
        if not loaded_data:
            self.save_file(new_int_mark, "IntMark")
        return new_int_mark

    def load_edge(self, data):
        """
        Method used to load the existing edge given in parameter
        :param data: The data file of the edge
        :return: True if the Edge has been correctly loaded, False otherwise
        """
        temp_itm_1 = self.get_itm(data['start marker'])
        temp_itm_2 = self.get_itm(data['end marker'])
        if temp_itm_1 is None or temp_itm_2 is None:
            return False

        new_edge = MapEdge(self.server.get(data['start marker']), self.server.get(data['end marker']), data['id'], data['weight'])
        temp_itm_1.list_of_edges.append(new_edge)
        temp_itm_2.list_of_edges.append(new_edge)

        self.edge_pub.publish(new_edge.edge_marker)
        self.edges_list.append(new_edge)
        self.map_graph.add_edge_to_graph(data['start marker'], data['end marker'], data['weight'])
        return True

    def create_edge(self, feedback):
        """
        Method that works in two times :
            - The first time the user click on "create edge" in the menu of the interactive marker, it saves that marker in memory
            - The second time, it looks if the edge already exists and if not it creates the Edge object, saves and publishes it
        :param feedback: InteractiveMarkerFeedback
        :return: -1 if the edge already exists
        """

        if not self.starting_point_created:
            self.start_temp_mark = self.get_itm(feedback.marker_name)
            self.starting_point_created = True
        else:
            end_temp_mark = self.get_itm(feedback.marker_name)
            if self.map_graph.is_neighbor(self.start_temp_mark.int_marker.name, end_temp_mark.int_marker.name):
                rospy.logwarn("Edge already exists, select an other couple of nodes")
                self.starting_point_created = False
                return -1
            current_edge = MapEdge(self.start_temp_mark.int_marker, end_temp_mark.int_marker)
            self.start_temp_mark.list_of_edges.append(current_edge)
            end_temp_mark.list_of_edges.append(current_edge)

            self.starting_point_created = False
            self.edges_list.append(current_edge)
            self.edge_pub.publish(current_edge.edge_marker)
            self.server.applyChanges()
            self.map_graph.add_edge_to_graph(self.start_temp_mark.int_marker.name, end_temp_mark.int_marker.name)
            self.save_file(current_edge, "Edge")

    # --------------------
    # Delete methods
    def delete_int_mark(self, feedback):
        """
        Delete the Interactive Marker from the graph, the server, the map and its data file
        :param feedback: InteractiveMarkerFeedback
        """
        itm = self.get_itm(feedback.marker_name)
        self.delete_all_edges(feedback)
        self.map_graph.delete_node_on_graph(itm.int_marker.name)
        self.server.erase(itm.int_marker.name)
        self.server.applyChanges()

        self.itm_list.remove(itm)
        if os.path.exists(self.CONFIG_PATH + itm.int_marker.name + '.json'):
            os.remove(self.CONFIG_PATH + itm.int_marker.name + '.json')

    def delete_edge(self, edge):
        """
        Delete the Edge object `edge` from the graph, the map and the two IntMark's list_of_edges which it's associated with and
        its data file
        :param edge: The Edge object to be deleted
        """
        self.map_graph.delete_edge_on_graph(edge.int_mark_1.name, edge.int_mark_2.name)
        itm1 = self.get_itm(edge.int_mark_1.name)
        itm1.list_of_edges.remove(edge)
        itm2 = self.get_itm(edge.int_mark_2.name)
        itm2.list_of_edges.remove(edge)

        edge.delete_edge()
        self.edges_list.remove(edge)
        self.server.applyChanges()
        temp_path = self.CONFIG_PATH + edge.edge_marker.ns + '_' + str(edge.edge_marker.id) + '.json'
        if os.path.exists(temp_path):
            os.remove(temp_path)

    def delete_all_edges(self, feedback):
        """
        Delete all the edges associated to the IntMark object
        :param feedback: InteractiveMarkerFeedback, used to retrieve the IntMark object
        """
        itm = self.get_itm(feedback.marker_name)
        for edge in reversed(itm.list_of_edges):
            self.delete_edge(edge)

    # --------------------
    # ROS Services and specific methods
    def add_itm_srv(self, req):
        """
        A ROS Service that uses the request `req` to create an ItM
        :param req: the request containing all the attributes to create an ItM
        :return:
        """
        data = {"name": req.new_itm.name,
                "description": req.new_itm.name,
                "type": "IntMark",
                "pose": {
                     "position": {
                         "x": req.new_itm.pose.position.x,
                         "y": req.new_itm.pose.position.y,
                         "z": req.new_itm.pose.position.z
                     },
                     "orientation": {
                         "x": req.new_itm.pose.orientation.x,
                         "y": req.new_itm.pose.orientation.y,
                         "z": req.new_itm.pose.orientation.z,
                         "w": req.new_itm.pose.orientation.w
                     }
                 }
                }
        self.save_file(self.add_itm(data, True), "IntMark")
        return AddItMResponse(True)

    def get_itm_srv(self, req):
        """
        A ROS Service that returns the name and pose of the IntMark object corresponding to the name in the request
        :param req: The request from the client, containing the name of the IntMark object to find
        :return: The name and pose of the IntMark object or None if it doesn't exists
        """
        itm = self.get_itm(req.name)
        if itm is None:
            return GetItMResponse("", None)
        return GetItMResponse(ItM(name=itm.int_marker.name, pose=itm.int_marker.pose))

    def add_edge_srv(self, req):
        """
        A ROS Service that adds an edge between the two Interactive markers in the request
        :param req: Contains the name of the two Interactive markers
        :return:
        """
        temp_itm_1 = self.get_itm(req.new_edge.first_node)
        temp_itm_2 = self.get_itm(req.new_edge.second_node)
        new_edge = MapEdge(self.server.get(temp_itm_1.int_marker.name), self.server.get(temp_itm_2.int_marker.name))
        temp_itm_1.list_of_edges.append(new_edge)
        temp_itm_2.list_of_edges.append(new_edge)

        self.edge_pub.publish(new_edge.edge_marker)
        self.edges_list.append(new_edge)
        self.map_graph.add_edge_to_graph(req.new_edge.first_node, req.new_edge.second_node)
        self.save_file(new_edge, "Edge")
        return AddEdgeResponse(True)

    def send_graph_srv(self, req):
        """
        A ROS Service to return an object of type Graph from the map_manager messages, containing ItM and Edge messages.
        :param req: The request is empty
        :return: The Graph object
        """
        graph_resp = Graph()
        for itm in self.itm_list:
            graph_resp.nodes.append(ItM(name=itm.int_marker.name, pose=itm.int_marker.pose))
        for _edge in self.edges_list:
            graph_resp.edges.append(Edge(first_node=_edge.int_mark_1.name, second_node=_edge.int_mark_2.name))
        return SendGraphResponse(graph_resp)

    def change_node_weight_srv(self, req):
        """
        For the moment, just a service to change the weight of edges associated with the node in the request
        :param req: Request of client
        :return:
        """
        if req.act == "reset":
            for edge in self.edges_list:
                edge.edge_weight = 1.0
                self.save_file(edge, "Edge")
                self.map_graph.change_edge_weight(edge.int_mark_1.name, edge.int_mark_2.name, 1.0)
        else:
            itm = self.get_itm(req.node)
            for edge in itm.list_of_edges:
                edge.edge_weight = req.weight
                self.save_file(edge, "Edge")
                self.map_graph.change_edge_weight(edge.int_mark_1.name, edge.int_mark_2.name, req.weight)

        return UpdateGraphResponse(True)

    def make_path_srv(self, req):
        """
        A ROS Service to return the list of interactive markers to go from one interactive marker to another.
        It uses the Dijkstra algorithm from the Networkx package.
        :param req: The request containing the two points in space
        :return: The list of the interactive markers between the start and the end
        """
        if self.current_map is None:
            self.current_map = self._getMap()

        itms_list_resp = []

        if req.mode == "fast_ros_plan":
            ros_make_plan = self.path(req.start, req.end, 0.0)
            result = self.check_itms_on_path(ros_make_plan.plan.poses)
            itm_near_start_point = result[0][0]
            itm_near_end_point = result[-1][0]
        else:
            if req.mode != "euclidian" and req.mode != "ros_plan":
                req.mode = "euclidian"
            res1 = self.find_nearest_itm(req.start.pose, req.mode)
            res2 = self.find_nearest_itm(req.end.pose, req.mode)
            itm_near_start_point = res1[0]
            itm_near_end_point = res2[0]

        path = self.map_graph.make_path(itm_near_start_point.int_marker.name, itm_near_end_point.int_marker.name)

        rospy.loginfo("path : {}".format(path))
        if path is None or itm_near_start_point is None or itm_near_end_point is None:
            return MakePathResponse(None)

        for _itm in path:
            map_itm = self.get_itm(_itm)
            itms_list_resp.append(ItM(name=_itm, pose=map_itm.int_marker.pose))
        return MakePathResponse(itms_list_resp)


    def find_nearest_itm(self, initial_point, mode, min_distance=0):
        """
        Returns the nearest IntMark object of the `initial_point` in the map
        :param mode: "EUCLIDIAN" to measure the euclidian distance between two points, otherwise it measure the
        distance taking account of obstacles
        :param min_distance: The minimal distance the IntMark object will be accepted
        :param initial_point: A geometry_msgs/Pose
        :return: The IntMark object and its distance from the point
        """
        _list = []

        if mode == "euclidian":
            for _itm in self.itm_list:
                point1 = [initial_point.position.x, initial_point.position.y, initial_point.position.z]
                point2 = [_itm.int_marker.pose.position.x, _itm.int_marker.pose.position.y,_itm.int_marker.pose.position.z]
                dist = distance.euclidean(point1, point2)
                if min_distance < dist:
                    _list.append([_itm, dist])
        else:
            ps1 = self.convert_to_pose_stamped(initial_point)
            for _itm in self.itm_list:
                ps2 = self.convert_to_pose_stamped(_itm.int_marker.pose)
                dist = self.check_distance(ps1, ps2)
                if min_distance < dist:
                    _list.append([_itm, dist])

        _list.sort(key=lambda _list: _list[1])
        return _list[0]

    def check_itms_on_path(self, _path):
        max_dist = 2
        itms_on_path = []
        for i in range(0, len(_path), 20):
            min_dist = float('inf')
            itm_close = None

            pt = _path[i]
            x_min = pt.pose.position.x - max_dist
            x_max = pt.pose.position.x + max_dist
            y_min = pt.pose.position.y - max_dist
            y_max = pt.pose.position.y + max_dist

            for _itm in self.itm_list:
                x = _itm.int_marker.pose.position.x
                y = _itm.int_marker.pose.position.y
                if x_min < x < x_max and y_min < y < y_max:
                    ps2 = self.convert_to_pose_stamped(_itm.int_marker.pose)
                    dist = self.check_distance(pt, ps2)
                    if dist < min_dist:
                        itm_close = _itm
                        min_dist = dist
            if itm_close is not None:
                itms_on_path.append([itm_close, min_dist])
        return itms_on_path

    def check_distance(self, point_1, point_2):
        """
        Return the minimal distance between the two points that avoid obstacles
        :param point_1: A geometry_msgs/PoseStamped
        :param point_2: A geometry_msgs/PoseStamped
        :return: The distance between the two points avoiding obstacles
        """
        _path = self.path(point_1, point_2, 0.0)
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
        tmp.header.frame_id = "map"
        tmp.header.stamp = rospy.Time.now()
        tmp.pose = pose

        return tmp

    # --------------------
    # Save/update and file modification methods
    def int_mark_process_feedback(self, feedback):
        """
        Method called when the InteractiveMarkerServer receives the information that an Interactive Marker is being moved
        :param feedback: InteractiveMarkerFeedback
        """
        temp_int_mark = self.get_itm(feedback.marker_name)
        for edge in temp_int_mark.list_of_edges:
            edge.update_edge(feedback)
            self.edge_pub.publish(edge.edge_marker)
        self.server.applyChanges()
        self.save_file(temp_int_mark, "IntMark")

    def save_file(self, obj, type_of_obj):
        """
        Save/update the data file of the IntMark or Edge object
        :param obj: The IntMark or Edge object that will be saved in a file
        :param type_of_obj: Type of the object parameter, either IntMark or Edge
        """
        if type_of_obj == "IntMark":
            with open(self.CONFIG_PATH + obj.int_marker.name + '.json', 'w+') as f:
                pose_dict = {"type": "IntMark",
                             "name": obj.int_marker.name,
                             "description": obj.int_marker.description,
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

        elif type_of_obj == "Edge":
            name = obj.edge_marker.ns + '_' + str(obj.edge_marker.id)
            with open(self.CONFIG_PATH + name + '.json', 'w+') as f:
                pose_dict = {"type": "Edge",
                             "name": name,
                             "id": obj.edge_marker.id,
                             "start marker": obj.int_mark_1.name,
                             "end marker": obj.int_mark_2.name,
                             "weight": obj.edge_weight
                             }
                f.write(json.dumps(pose_dict, indent=4))

        else:
            rospy.logwarn("Type of object to save in a file not recognized")

    def created_file(self, filepath):
        """
        Method called when a file is created to check if it's an IntMark or Edge data file. Then it checks if the object
        of the corresponding type exists and if not it calls the method to create it
        :param filepath: Path of the file created
        """
        with open(filepath, 'r') as f:
            data = eval(json.dumps(json.load(f)))
            if data['type'] == "IntMark":
                if self.get_itm(data['name']) is None:
                    self.add_itm(data, True)
            elif data['type'] == "Edge":
                if self.get_edge(data['id']) is None:
                    self.load_edge(data)
            else:
                rospy.logwarn("Data type not recognized")

    def modified_file(self, filepath):
        """
        Method called when a file is modified to check if it's an IntMark or Edge data file. Then it retrieves the corresponding
        object in the list of the map manager and checks if it needs an update on the map
        :param filepath: Path of the file modified
        """
        with open(filepath, 'r') as f:
            data = eval(json.dumps(json.load(f)))  # FIXME: bug load json
            if data['type'] == "IntMark":
                temp_itm = self.get_itm(data['name'])
                self.compare_data(temp_itm.int_marker.description, data['description'], True)
                self.compare_data(temp_itm.int_marker.pose.position.x, data['pose']['position']['x'], True)
                self.compare_data(temp_itm.int_marker.pose.position.y, data['pose']['position']['y'], True)
                self.compare_data(temp_itm.int_marker.pose.position.z, data['pose']['position']['z'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.x, data['pose']['orientation']['x'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.y, data['pose']['orientation']['y'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.z, data['pose']['orientation']['z'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.w, data['pose']['orientation']['w'], True)
                self.server.applyChanges()
            elif data['type'] == "Edge":
                pass
            else:
                rospy.logwarn("Data type not recognized")

    @staticmethod
    def compare_data(data_1, data_2, change_data=False):
        """
        Method that compares `data_1` and `data_2` and changed `data_1` to the value of `data_2` if they are different and that
        change_data is True
        :param data_1: First data to compare
        :param data_2: Second data to compare
        :param change_data: Boolean that allows the changement of the value of data_1 to data_2's value or not
        :return: True if both data are equal, False otherwise
        """
        if data_1 != data_2 and change_data is True:
            data_1 = data_2
            return False
        elif data_1 != data_2 and change_data is False:
            return False
        return True

    def deleted_file(self, filepath):
        """
        Method called when a file is deleted to check if it's an IntMark or Edge data file. Then it retrieves the corresponding
        object in the list of the map manager and deletes it if it still exists
        :param filepath: Path of the deleted file
        """
        with open(filepath, 'r') as f:
            data = eval(json.dumps(json.load(f)))
            if data['type'] == "IntMark":
                itm = self.get_itm(data['name'])
                if itm is not None:
                    self.map_graph.delete_node_on_graph(itm.int_marker.name)
                    for edge in reversed(itm.list_of_edges):
                        self.delete_edge(edge)
                    self.server.erase(itm.int_marker.name)
                    self.server.applyChanges()
                    self.itm_list.remove(itm)
            elif data['type'] == "Edge":
                edge = self.get_edge(data['id'])
                if edge is not None:
                    self.delete_edge(edge)
            else:
                rospy.logwarn("Data type not recognized")


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
        if datetime.now() - self.last_modified < timedelta(seconds=1):
            return
        self.last_modified = datetime.now()
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.modified_file(event.src_path)

    def on_deleted(self, event):
        """
        Method called when a file is deleted in the CONFIG_PATH folder
        :param event: Event that happens to a file
        """
        rospy.loginfo('Event type: {} file {}'.format(event.event_type, os.path.basename(event.src_path)))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            path = "/home/damien/.local/share/Trash/files/" + os.path.basename(event.src_path)  # TODO: changer path pour eviter /home/damien
            if os.path.exists(path):
                self.map_mng.deleted_file(path)


class MapGraph:
    """
    Class for the creation of a Graph object, which contains a Networkx graph that draws all the Interactive Markers as
    the nodes and the Edges as the edges
    """
    def __init__(self):
        """
        Initialize the Networkx Graph object
        """
        self.graph = nx.Graph()

    def add_marker_to_graph(self, _node):
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
        for n in self.graph.neighbors(node_2):
            if node_1 == n:
                return True
        return False

    def make_path(self, start_node, end_node):
        """
        Method that returns the list of nodes (interactive markers) to go from `start_node` to `end_node`.
        It uses the Dijkstra algorithm from the Networkx package.
        :param start_node: The starting node of the path
        :param end_node: The ending node of the path
        :return: The list of the interactive markers between the start and the end if there's no errors
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


class MapItM:
    """
    Class to create an IntMark object (an ItM) containing an Interactive Marker from the visualization_msgs package and some
    other attributes and methods
    """
    current_marker_id = 0  # ID of markers that increments each time an IntMark object is created, so they're unique
    map_mng = ""

    def __init__(self):
        """
        Initialization of a new IntMark object
        """
        self.int_marker = InteractiveMarker()
        self.menu_handler = MenuHandler()
        self.list_of_edges = []  # List of all Edge objects associated with the IntMark object

    @staticmethod
    def init_map_mng(_map_mng):
        """
        Initialize the map manager attribute of the class, so the IntMark objects can call methods from MapMng
        :param _map_mng: The map manager created at the beginning of the program
        """
        MapItM.map_mng = _map_mng

    def make_inter_marker(self, data, loaded_data=False):
        """
        Create the Interactive Marker, its controls and saves it in a file if it is a new IntMark object
        :param data: The data used to create the Interactive Marker, either a Point Stamped message or a dictionnary
        containing the position of the marker and its other attributes
        :param loaded_data: Tells the method which type of data is received, False if it is a new IntMark object so `data`
        is a Point Stamped message and True if it is a loaded data from the json/ folder so `data` is a dictionnary
        """
        self.int_marker.header.frame_id = "map"
        self.int_marker.scale = 1

        if not loaded_data:
            self.int_marker.name = "ItM" + str(MapItM.current_marker_id)
            self.int_marker.description = self.int_marker.name
            self.int_marker.header.stamp = data.header.stamp
            self.int_marker.pose.position = data.point
            self.int_marker.pose.orientation.w = 1.0
            MapItM.current_marker_id += 1
        else:
            self.int_marker.name = data['name']
            self.int_marker.description = data['name']
            self.int_marker.header.stamp = rospy.Time.now()
            self.int_marker.pose.position.x = data['pose']['position']['x']
            self.int_marker.pose.position.y = data['pose']['position']['y']
            self.int_marker.pose.position.z = data['pose']['position']['z']
            self.int_marker.pose.orientation.x = data['pose']['orientation']['x']
            self.int_marker.pose.orientation.y = data['pose']['orientation']['y']
            self.int_marker.pose.orientation.z = data['pose']['orientation']['z']
            self.int_marker.pose.orientation.w = data['pose']['orientation']['w']
            if int(data['name'][3:]) >= MapItM.current_marker_id:
                MapItM.current_marker_id = int(data['name'][3:]) + 1
        self.make_controls()
        self.add_menu()

    def make_controls(self):
        """
        Method making all the controls of the Interactive marker so it can be moved around the map and a menu can be open
        by right clicking on the arrow marker
        """
        # interactive control containing the menu and the arrow marker
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(self.make_marker())
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

    def make_marker(self):
        """
        Make a green arrow marker that represents the interactive markers
        :return: The Marker object
        """
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = self.int_marker.scale * 0.5
        marker.scale.y = self.int_marker.scale * 0.2
        marker.scale.z = self.int_marker.scale * 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def add_menu(self):
        """
        Add a menu to the Interactve marker that allows some actions on it : create/delete edges and delete the marker
        """
        self.menu_handler.insert("Create an edge", callback=MapItM.map_mng.create_edge)
        self.menu_handler.insert("Delete all edges", callback=MapItM.map_mng.delete_all_edges)
        self.menu_handler.insert("Delete marker", callback=MapItM.map_mng.delete_int_mark)


class MapEdge:
    """
    Class to create an Edge object which represents a link between two interactive markers
    """
    current_id = 0  # Id given to the next edge created if it is a new edge, increasing by one so all edges have a different Id

    def __init__(self, _itm1, _itm2, _id=None, _weight=None):
        """
        Initialization of a new edge created by the two Interactive Markers objects `_itm1` and `_itm2`
        :param _itm1: The first node of the edge
        :param _itm2: The second node of the edge
        :param _id: If the edge is loaded it already has an Id. Otherwise the value of `_id` is None so the method will take the value of current_id
        """

        self.int_mark_1 = _itm1
        self.int_mark_2 = _itm2
        self.edge_marker = Marker()
        self.start_point = Point()
        self.end_point = Point()

        # Marker line
        self.edge_marker.header.frame_id = "map"
        self.edge_marker.header.stamp = rospy.Time.now()
        self.edge_marker.ns = "Edge"
        self.edge_marker.type = Marker.LINE_STRIP
        self.edge_marker.action = Marker.ADD
        self.edge_marker.color.r = 1.0
        self.edge_marker.color.a = 1.0
        self.edge_marker.scale.x = 0.1

        # Starting point of the line
        self.start_point.x = self.int_mark_1.pose.position.x
        self.start_point.y = self.int_mark_1.pose.position.y
        self.start_point.z = self.int_mark_1.pose.position.z
        self.edge_marker.points.append(self.start_point)

        # Ending point of the line
        self.end_point.x = self.int_mark_2.pose.position.x
        self.end_point.y = self.int_mark_2.pose.position.y
        self.end_point.z = self.int_mark_2.pose.position.z
        self.edge_marker.points.append(self.end_point)

        if _id is None:
            # It means it's a new edge so we give it an Id and save it in the CONFIG_PATH folder
            self.edge_marker.id = MapEdge.current_id
            MapEdge.current_id += 1
        else:
            # It's an edge being loaded
            self.edge_marker.id = _id
            if _id >= MapEdge.current_id:
                # It sets the current_id to the maximum id of the loaded edges
                MapEdge.current_id = _id + 1

        if _weight is None:
            self.edge_weight = 1.0
        else:
            self.edge_weight = _weight

    def update_edge(self, feedback):
        """
        Update the position of the edge according to the two markers that defines it.
        This method is called by the processFeedback method of the interactive marker
        :param feedback: The interactive marker moving on the map
        """
        if feedback.marker_name == self.int_mark_1.name:
            self.edge_marker.points[0] = feedback.pose.position
        elif feedback.marker_name == self.int_mark_2.name:
            self.edge_marker.points[1] = feedback.pose.position
        else:
            rospy.logerr("Error, the marker is not associated with the edge")

    def delete_edge(self):
        """
        Delete the marker of the edge from the map
        """
        self.edge_marker.action = 2


if __name__ == '__main__':
    rospy.init_node("pepper_interactive_marker")

    default_value = 'map_test/'
    dir_name = rospy.get_param("~confPath", default_value)
    _cfgpath = os.path.join(os.path.dirname(__file__), '../json/') + dir_name
    if not os.path.isdir(_cfgpath):
        rospy.logerr("Directory {} does not exist, create it before executing the program".format(_cfgpath))
    else:
        manager = MapMng(_cfgpath)
        time.sleep(1)  # HACK - Wait for ROS Subscribers to listen to the edge publisher
        manager.load()
        rospy.spin()
