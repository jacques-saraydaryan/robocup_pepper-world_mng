#!/usr/bin/env python
# coding: utf8

import json
import os
import time
import fnmatch
import networkx as nx
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from threading import Thread

import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.menu_handler import MenuHandler
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from map_manager.srv import MakePathService, MakePathServiceResponse

CONFIG_PATH = ""  # Global variable representing the path to the folder where data is saved


class MapMng:
    """
    Class for the management of the map of the environment of the robot, it makes the link between the objects of the
    other classes (IntMark, Graph, Edges)
    """
    start_temp_mark = ""  # Variable that keeps track of the first IntMark object used to create an edge
    starting_point_created = False  # Variable for the creation of a new edge so the create_edge method has a different behavior following this value

    def __init__(self, cfg_path):
        """
        Initializes the server for the management of interactive markers, the publisher for the display of edges, the
        subscriber to listen to the Clicked Point tool of RViz, an observer and an event handler to watch the
        modification of the files
        :param cfg_path: Path to save IntMark and Edges data files
        """
        global CONFIG_PATH
        CONFIG_PATH = cfg_path
        self.itm_list = []  # List of all IntMark objects
        self.edges_list = []  # List of all Edge objects

        # Observer and event handler to watch any modifications of the data files
        self.event_handler = MyHandler(self)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, path=CONFIG_PATH, recursive=False)
        self.observer.start()

        # Initialization of objects and variables, load of existing markers and edges and subscritpion to the
        # clicked_point topic
        IntMark.init_map_mng(self)
        self.server = InteractiveMarkerServer("pepper_inter_markers")
        self.edge_pub = rospy.Publisher("/interactive_marker_edges", Marker, queue_size=10)
        time.sleep(1)  # HACK - Wait for ROS1 Subscribers to listen to the edge publisher
        self.map_graph = Graph()
        self.load(CONFIG_PATH)
        self.subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.add_itm)

        rospy.spin()
        if rospy.is_shutdown():
            rospy.loginfo("Exiting program")
            self.subscriber.unregister()
            self.edge_pub.unregister()
            self.map_graph.join()

    def get_itm(self, itm_name):
        """
        Return the IntMark object corresponding to the name given in parameter
        :param itm_name: Name of the IntMark object to find in the list
        :return: The IntMark object if it exists, None otherwise
        """
        # [itm for itm in self.itm_list if itm.int_marker.name==itm_name][0]  # List comprehension version
        for itm in self.itm_list:
            if itm.int_marker.name == itm_name:
                return itm
        rospy.loginfo("IntMark object of name {} not found".format(itm_name))
        return None

    def get_edge(self, edge_id):
        """
        Return the Edge object corresponding to the id given in parameter
        :param edge_id: Id of the Edge object to find in the list
        :return: The Edge object if it exists, None otherwise
        """
        # [edge for edge in self.edges_list if edge.edge_marker.id==edge_id][0]  # List comprehension version
        for edge in self.edges_list:
            if edge.edge_marker.id == edge_id:
                return edge
        rospy.loginfo("Edge object of id {} not found".format(edge_id))
        return None

    def load(self, _cfgpath):
        """
        Method that loads all existing markers and puts them in the Interactive Marker Server and then the edges between
        the markers at the launch of the program
        :param _cfgpath: The path of the folder where the IntMark and Edge objects are saved
        """
        dirs = os.listdir(_cfgpath)
        edges_files = []

        for filename in dirs:
            with open(_cfgpath + filename, 'r') as f:
                data = json.load(f)
                data['name'] = data['name'].encode("utf8")
                if data['type'] == "IntMark":
                    rospy.loginfo("Loading IntMark {}".format(data['name']))
                    self.add_itm(data, True)
                elif data['type'] == "Edge":
                    edges_files.append(filename)
                else:
                    rospy.logwarn("Data type not recognized")

        for filename in edges_files:
            with open(_cfgpath + filename, 'r') as f:
                data = json.load(f)
                rospy.loginfo("Loading Edge {}".format(data['name']))
            if not self.load_edge(data):
                rospy.logwarn("IntMark of the Edge not found, deleting Edge file")
                os.remove(_cfgpath + filename)
            edges_files.remove(filename)

        rospy.loginfo("All Interactive Markers and Edges have been loaded")

    def add_itm(self, data, loaded_data=False):
        """
        Method called to create an IntMark object or to load an existing one
        :param data: The data on the IntMark object
        :param loaded_data: True if it's a loaded IntMark so data is a dictionnary from the .json file, False otherwise
        so data is a Clicked Point message
        """
        new_int_mark = IntMark()
        new_int_mark.make_inter_marker(data, loaded_data)

        self.server.insert(new_int_mark.int_marker, self.int_mark_process_feedback)
        new_int_mark.menu_handler.apply(self.server, new_int_mark.int_marker.name)
        self.server.applyChanges()

        self.itm_list.append(new_int_mark)
        self.map_graph.add_marker_to_graph(new_int_mark.int_marker.name)
        if not loaded_data:
            self.save_file(new_int_mark, "IntMark")

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

        new_edge = Edge(self.server.get(data['start marker']), self.server.get(data['end marker']), data['id'])
        temp_itm_1.list_of_edges.append(new_edge)
        temp_itm_2.list_of_edges.append(new_edge)

        self.server.applyChanges()
        self.edge_pub.publish(new_edge.edge_marker)
        self.edges_list.append(new_edge)
        self.map_graph.add_edge_to_graph(data['start marker'], data['end marker'])
        return True

    def create_edge(self, feedback):
        """
        Method that works in two times :
            - The first time the user click on "create edge" in the menu of the interactive marker, it saves that marker in memory
            - The second time, it looks if the edge already exists and if not it creates the Edge object, saves and publishes
            it and create a submenu on each marker containing the name of the other one in the "Delete edge to" menu
        :param feedback: InteractiveMarkerFeedback
        :return: -1 if the edge already exists
        """

        if not MapMng.starting_point_created:
            MapMng.start_temp_mark = self.get_itm(feedback.marker_name)
            MapMng.starting_point_created = True
        else:
            end_temp_mark = self.get_itm(feedback.marker_name)
            if self.map_graph.is_neighbor(MapMng.start_temp_mark.int_marker.name, end_temp_mark.int_marker.name):
                rospy.logwarn("Edge already exists, select an other couple of nodes")
                MapMng.starting_point_created = False
                return -1
            current_edge = Edge(MapMng.start_temp_mark.int_marker, end_temp_mark.int_marker)
            MapMng.start_temp_mark.list_of_edges.append(current_edge)
            end_temp_mark.list_of_edges.append(current_edge)

            MapMng.starting_point_created = False
            self.edges_list.append(current_edge)
            self.edge_pub.publish(current_edge.edge_marker)
            self.server.applyChanges()
            self.map_graph.add_edge_to_graph(MapMng.start_temp_mark.int_marker.name, end_temp_mark.int_marker.name)
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
        if os.path.exists(CONFIG_PATH + itm.int_marker.name + '.json'):
            os.remove(CONFIG_PATH + itm.int_marker.name + '.json')

    def delete_edge(self, edge):
        """
        Delete the Edge object edge from the graph, the map and the two IntMark's list_of_edges which it's associated with and
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
        temp_path = CONFIG_PATH + edge.edge_marker.ns + '_' + str(edge.edge_marker.id) + '.json'
        if os.path.exists(temp_path):
            os.remove(temp_path)

    def delete_all_edges(self, feedback):
        """
        Delete all the edges associated to the IntMark object
        :param feedback: InteractiveMarkerFeedback, used to retrieve the IntMark object
        """
        itm = self.get_itm(feedback.marker_name)
        for edge in itm.list_of_edges:
            self.delete_edge(edge)

    # --------------------
    # Update methods
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

    @staticmethod
    def save_file(obj, type_of_obj):
        """
        Save/update the data file of the IntMark or Edge object
        :param obj: The IntMark or Edge object that will be saved in a file
        :param type_of_obj: Type of the object parameter, either IntMark or Edge
        """
        if type_of_obj == "IntMark":
            with open(CONFIG_PATH + obj.int_marker.name + '.json', 'w+') as f:
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
            with open(CONFIG_PATH + name + '.json', 'w+') as f:
                pose_dict = {"type": "Edge",
                             "name": name,
                             "id": obj.edge_marker.id,
                             "start marker": obj.int_mark_1.name,
                             "end marker": obj.int_mark_2.name
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
            data = json.load(f)
            if data['type'] == "IntMark":
                if self.get_itm(data['name']) is None:
                    rospy.loginfo("Creating IntMark")
                    self.add_itm(data, True)
            elif data['type'] == "Edge":
                if self.get_edge(data['id']) is None:
                    rospy.loginfo("Creating Edge")
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
            print("path : {}".format(filepath))
            data = json.load(f)
            if data['type'] == "IntMark":
                rospy.loginfo("IntMark file modified")
                temp_itm = self.get_itm(data['name'])
                self.compare_data(temp_itm.int_marker.description, data['description'], True)
                self.compare_data(temp_itm.int_marker.pose.position.x, data['pose']['position']['x'], True)
                self.compare_data(temp_itm.int_marker.pose.position.y, data['pose']['position']['y'], True)
                self.compare_data(temp_itm.int_marker.pose.position.z, data['pose']['position']['z'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.x, data['pose']['orientation']['x'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.y, data['pose']['orientation']['y'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.z, data['pose']['orientation']['z'], True)
                self.compare_data(temp_itm.int_marker.pose.orientation.w, data['pose']['orientation']['w'], True)
                rospy.loginfo("IntMark updated")
                self.server.applyChanges()
            elif data['type'] == "Edge":
                rospy.logwarn("Edge object cannot be modified")
            else:
                rospy.logwarn("Data type not recognized")


    @staticmethod
    def compare_data(data_1, data_2, change_data=False):
        """
        Method that compares data_1 and data_2 and changed data_1 to the value of data_2 if they are different and that
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
            print("Reading deleted file")
            data = json.load(f)
            if data['type'] == "IntMark":
                itm = self.get_itm(data['name'])
                if itm is not None:
                    self.map_graph.delete_node_on_graph(itm.int_marker.name)
                    for edge in itm.list_of_edges:
                        itm.list_of_edges.remove(edge)
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

    def on_created(self, event):
        """
        Method called when a file is created in the CONFIG_PATH folder
        :param event: Event that happens to a file
        :return:
        """
        print('Event type: {} file {}'.format(event.event_type, event.src_path))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.created_file(event.src_path)

    def on_modified(self, event):
        """
        Method called when a file is modified in the CONFIG_PATH folder
        :param event: Event that happens to a file
        :return:
        """
        print('Event type: {} file {}'.format(event.event_type, event.src_path))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            self.map_mng.modified_file(event.src_path)

    def on_deleted(self, event):
        """
        Method called when a file is deleted in the CONFIG_PATH folder
        :param event: Event that happens to a file
        :return:
        """
        print('Event type: {} file {}'.format(event.event_type, event.src_path))
        if fnmatch.fnmatch(event.src_path, "*.json"):
            path = "/home/damien/.local/share/Trash/files/" + os.path.basename(event.src_path)
            if os.path.exists(path):
                self.map_mng.deleted_file(path)


class Graph(Thread):
    """
    Class for the creation of a Graph object, which contains a Networkx graph that draws all the Interactive Markers as
    nodes and the Edges as edges, and a service that returns a path from a node to another based on Dijkstra algorithm
    """
    def __init__(self):
        """
        Initialize the Graph object and its attributes
        """
        Thread.__init__(self)
        self.start()
        self.graph = nx.Graph()
        self.navigation_path_service = rospy.Service("make_path_from_graph", MakePathService, self.make_path)

    def add_marker_to_graph(self, _node):
        """
        Add a node _node to the graph
        :param _node: Name of the node to be added (usually it's the name of the interactive marker)
        """
        self.graph.add_node(_node)

    def add_edge_to_graph(self, first_node, second_node):
        """
        Add an edge between first_node and second_node to the graph
        :param first_node: Name of the first node of the edge
        :param second_node: Name of the second node of the edge
        """
        self.graph.add_edge(first_node, second_node)

    def delete_node_on_graph(self, _node):
        """
        Delete the node _node from the graph
        :param _node: Name of the node to be removed
        """
        if self.graph.has_node(_node):
            self.graph.remove_node(_node)

    def delete_edge_on_graph(self, first_node, second_node):
        """
        Remove the edge of the graph connecting first_node and second_node
        :param first_node: Name of the first node of the edge
        :param second_node: Name of the second node of the edge
        """
        if self.is_neighbor(first_node, second_node):
            self.graph.remove_edge(first_node, second_node)

    def make_path(self, req):
        """
        A ROS Service to return the list of interactive markers to go from one interactive marker to another.
        It uses the Dijkstra algorithm from the Networkx package
        :param req: The request containing the two interactive markers representing the start and the end of the path
        :return: The list of the interactive markers between the start and the end
        """
        path = MakePathServiceResponse(nx.dijkstra_path(self.graph, req.start, req.end))
        return path

    def is_neighbor(self, node_1, node_2):
        """
        A method to test if node_1 and node_2 are connected by an edge
        :param node_1: First node of the graph
        :param node_2: Second node of the graph
        :return: True if they are connected, False otherwise
        """
        for n in self.graph.neighbors(node_2):
            if node_1 == n:
                return True
        return False


class IntMark:
    """
    Class to create an IntMark object containing an Interactive Marker from the visualization_msgs package and some
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
        IntMark.map_mng = _map_mng

    def get_edge(self, _itm):
        """
        Return the Edge object corresponding to the two interactive markers (self and _itm)
        :param _itm: The second interactive marker of the edge
        :return: The Edge object or None if no edge exists between self and _itm
        """
        for edge in self.list_of_edges:
            if self.int_marker.name == edge.int_mark_1.name and _itm == edge.int_mark_2.name:
                return edge
            elif self.int_marker.name == edge.int_mark_2.name and _itm == edge.int_mark_1.name:
                return edge
        rospy.logwarn("No edge found for the two interactive markers")
        return None

    def make_inter_marker(self, data, loaded_data=False):
        """
        Create the Interactive Marker, its controls and saves it in a file if it is a new IntMark object
        :param data: The data used to create the Interactive Marker, either a Point Stamped message or a dictionnary
        containing the position of the marker and its other attributes
        :param loaded_data: Tells the method which type of data is received, False if it is a new IntMark object so data
        is a Point Stamped message and True if it is a loaded data from the json/ folder so data is a dictionnary
        """
        self.int_marker.header.frame_id = "map"
        self.int_marker.scale = 1

        if not loaded_data:
            self.int_marker.name = "ItM" + str(IntMark.current_marker_id)
            self.int_marker.description = self.int_marker.name
            self.int_marker.header.stamp = data.header.stamp
            self.int_marker.pose.position = data.point
            IntMark.current_marker_id += 1
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
            if int(data['name'][3:]) >= IntMark.current_marker_id:
                IntMark.current_marker_id = int(data['name'][3:]) + 1
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
        self.menu_handler.insert("Create an edge", callback=IntMark.map_mng.create_edge)
        self.menu_handler.insert("Delete all edges", callback=IntMark.map_mng.delete_all_edges)
        self.menu_handler.insert("Delete marker", callback=IntMark.map_mng.delete_int_mark)


class Edge:
    """
    Class to create an Edge object which represents a link between two interactive markers
    """
    current_id = 0  # Id given to the next edge created if it is a new edge, increasing by one so all edges have a
    # different Id

    def __init__(self, _itm1, _itm2, _id=None):
        """
        Initialization of a new edge created by the two Interactive Markers objects in parameter (not IntMark objects)
        :param _itm1: The first node of the edge
        :param _itm2: The second node of the edge
        :param _id: If the edge is loaded it already has an Id. Otherwise the value of _id is None so the method will take the value of current_id
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
        self.edge_marker.scale.x = 0.2

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
            self.edge_marker.id = Edge.current_id
            Edge.current_id += 1
        else:
            # It's an edge being loaded
            self.edge_marker.id = _id
            if _id >= Edge.current_id:
                # It sets the current_id to the maximum id of the loaded edges
                Edge.current_id = _id + 1

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
    default_value = "/home/damien/pepper_ws/src/robocup-main/robocup_pepper-world_mng/map_manager/json/"
    rospy.init_node("pepper_interactive_marker")
    _cfgpath = rospy.get_param("~confPath", default_value)

    manager = MapMng(_cfgpath)
