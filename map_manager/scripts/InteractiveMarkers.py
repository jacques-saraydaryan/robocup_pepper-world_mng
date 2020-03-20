#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *
import json
import os
import networkx as nx
import fnmatch
import matplotlib.pyplot as plt

now = 0  # Global variable for rospy.Time.now()
CONFIG_PATH = ""  # Global variable representing the path to the folder where data is saved
server = ""  # Server that contains all interactive markers and publishes them for Rviz
subscriber = ""  # Subscriber to the /clicked_point topic, it's the one used by the Publish Point tool on Rviz
edge_pub = ""  # Publishes all the markers of the class Edge whenever there's a modification, creation of a new edge etc...
current_edge = ""  # Temporary variable that keeps track of the current edge being created
temp_mark = ""  # Temporary variable that keeps track of the first marker used to create an edge


# Class for the management of the interactive markers in the map, creates objects of the class IntMark and edges of the class Edge between those objects
class MapMng:

    def __init__(self, _cfgpath):
        global CONFIG_PATH, server, subscriber, edge_pub
        self.graph = nx.Graph()
        server = InteractiveMarkerServer("pepper_inter_markers")
        CONFIG_PATH = _cfgpath

        edge_pub = rospy.Publisher("/interactive_marker_edges", Marker, queue_size=10)
        subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.inter_marker)
        rospy.spin()

    @staticmethod
    def inter_marker(data):
        IntMark(data)

    def load(self):
        global CONFIG_PATH, server
        dirs = os.listdir(CONFIG_PATH)

        for fileName in dirs:
            if fnmatch.fnmatch(fileName, 'ItM*.json'):
                with open(CONFIG_PATH + fileName, 'r') as f:
                    data = json.load(f)
                    self.graph.add_node(data['name'])
        for fileName in dirs:
            if fnmatch.fnmatch(fileName, 'Edge_*_*.json'):
                with open(CONFIG_PATH + fileName, 'r') as f:
                    data = json.load(f)
                    self.graph.add_edge(data['start marker'], data['end marker'])

        plt.subplot(111)
        nx.draw(self.graph, with_labels=True)
        plt.show()


# Class for the management of Interactive Markers into Rviz for the navigation of Pepper
class IntMark:
    current_marker_id = 0  # ID of markers that increments each time the user creates one, so they're unique
    starting_point_created = False

    def __init__(self, data):
        self.int_marker = InteractiveMarker()
        self.menu_handler = MenuHandler()

        self.list_of_edges_started = []  # List of all edges of which the interactive marker is the beginning point
        self.list_of_edges_ended = []  # List of all edges of which the interactive marker is the ending point

        self.add_marker(data)
        IntMark.current_marker_id += 1

    ###
    #   Methods for the creation of an interactive marker whenever the user uses the Publish Point tool on Rviz on a "map" frame :
    #       add_marker : create an interactive marker with the controls associated for the movement in the xy plane
    #       make_control : add the main control to the interactive marker that allows the user to click on it to open the menu
    #       make_marker : create the arrow marker on Rviz
    #       save_marker : save the properties of the interactive marker in a .json file
    #       update_marker : update the file whenever the marker moves in the map
    #       marker_menu : create the entries of the menu
    #       create_edge : create an edge of type LINE_STRIP marker between two interactive markers
    #       process_feedback : gives feedback in the terminal on what happens to the interactive markers
    ###

    def add_marker(self, data):
        global server

        self.int_marker.header.frame_id = "map"
        self.int_marker.header.stamp = rospy.Time.now()
        self.int_marker.pose.position = data.point
        self.int_marker.scale = 1

        self.int_marker.name = "ItM" + str(IntMark.current_marker_id)

        # insert a box
        self.make_control()

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

        # Apply changes to the server and save the coordinates of the marker in a .json file
        self.marker_menu(self.menu_handler)
        server.insert(self.int_marker, self.processFeedback)
        self.menu_handler.apply(server, self.int_marker.name)
        self.save_marker()
        server.applyChanges()

    def make_control(self):
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(self.make_marker())
        self.int_marker.controls.append(control)
        return control

    def make_marker(self):
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

    def marker_menu(self, mh):
        mh.insert("Create an edge", callback=self.create_edge)
        mh.insert("Delete all edges", callback=self.delete_all)
        mh.insert("Delete marker", callback=self.delete_marker)

    def create_edge(self, feedback):
        global server, now, current_edge, temp_mark
        if not IntMark.starting_point_created:
            now = rospy.Time.now()
            current_edge = Edge(server.get(feedback.marker_name))
            IntMark.starting_point_created = True
            temp_mark = self
        else:
            current_edge.closing_edge(server.get(feedback.marker_name))
            self.list_of_edges_ended.append(current_edge)
            temp_mark.list_of_edges_started.append(current_edge)
            IntMark.starting_point_created = False
            server.applyChanges()

    def delete_marker(self, feedback):
        global server, CONFIG_PATH
        self.delete_all(None)
        server.erase(feedback.marker_name)
        server.applyChanges()
        if os.path.exists(CONFIG_PATH + feedback.marker_name + '.json'):
            os.remove(CONFIG_PATH + feedback.marker_name + '.json')

    def delete_all(self, feedback):
        global server, edge_pub
        for edge in self.list_of_edges_started:
            self.list_of_edges_started.remove(edge)
            edge.delete_edge()
        for edge in self.list_of_edges_ended:
            self.list_of_edges_ended.remove(edge)
            edge.delete_edge()
        server.applyChanges()

    # Methods to report feedback, save and update data
    def processFeedback(self, feedback):
        global server

        self.update_marker(feedback)
        for edge in self.list_of_edges_started:
            edge.update_edge(feedback, 0)
        for edge in self.list_of_edges_ended:
            edge.update_edge(feedback, 1)

        server.applyChanges()

    def save_marker(self):
        with open(CONFIG_PATH + self.int_marker.name + '.json', 'w+') as f:
            pose_dict = {"name": self.int_marker.name,
                         "pose": {
                             "position": {
                                "x": self.int_marker.pose.position.x,
                                "y": self.int_marker.pose.position.y,
                                "z": self.int_marker.pose.position.z
                             },
                             "orientation": {
                                "x": self.int_marker.pose.orientation.x,
                                "y": self.int_marker.pose.orientation.y,
                                "z": self.int_marker.pose.orientation.z,
                                "w": self.int_marker.pose.orientation.w
                             },
                         },
                         }
            f.write(json.dumps(pose_dict, indent=4, sort_keys=True))
        rospy.loginfo('Successfully saved the Interactive Marker : {}'.format(self.int_marker.name))


    @staticmethod
    def update_marker(feedback):

        with open(CONFIG_PATH + feedback.marker_name + '.json', 'w+') as f:
            pose_dict = {"name": feedback.marker_name,
                         "pose": {
                             "position": {
                                 "x": feedback.pose.position.x,
                                 "y": feedback.pose.position.y,
                                 "z": feedback.pose.position.z
                             },
                             "orientation": {
                                 "x": feedback.pose.orientation.x,
                                 "y": feedback.pose.orientation.y,
                                 "z": feedback.pose.orientation.z,
                                 "w": feedback.pose.orientation.w
                             },
                         },
                         }
            f.write(json.dumps(pose_dict, indent=4, sort_keys=True))


# Class to create an Edge object which represents a link between two interactive markers
class Edge:
    edges_list = []

    def __init__(self, _itm1):
        self.int_mark_1 = _itm1
        self.edge_marker = Marker()
        self.start_point = Point()
        self.end_point = Point()

        self.opening_edge(_itm1)

    def opening_edge(self, _itm1):
        global now

        self.edge_marker.header.frame_id = "map"
        self.edge_marker.header.stamp = now
        self.edge_marker.type = Marker.LINE_STRIP
        self.edge_marker.action = Marker.ADD
        self.edge_marker.color.r = 1.0
        self.edge_marker.color.a = 1.0
        self.edge_marker.id = 1
        self.edge_marker.scale.x = 0.2

        self.start_point.x = self.int_mark_1.pose.position.x
        self.start_point.y = self.int_mark_1.pose.position.y
        self.start_point.z = self.int_mark_1.pose.position.z


        self.edge_marker.points.append(self.start_point)

    def closing_edge(self, _itm2):
        global edge_pub
        self.int_mark_2 = _itm2
        self.end_point.x = self.int_mark_2.pose.position.x
        self.end_point.y = self.int_mark_2.pose.position.y
        self.end_point.z = self.int_mark_2.pose.position.z

        self.edge_marker.points.append(self.end_point)

        self.edge_marker.ns = "Edge_" + self.int_mark_1.name[3:] + "_" + self.int_mark_2.name[3:]
        self.edges_list.append(self.edge_marker)
        for edge in self.edges_list:
            edge_pub.publish(edge)
        self.save_edge()

    def update_edge(self, feedback, mode):
        global edge_pub
        if mode == 0:
            self.edge_marker.points[0] = feedback.pose.position
        elif mode == 1:
            self.edge_marker.points[1] = feedback.pose.position
        edge_pub.publish(self.edge_marker)

    def delete_edge(self):
        global edge_pub
        self.edge_marker.action = 2
        for edge in self.edges_list:
            edge_pub.publish(edge)
        if os.path.exists(CONFIG_PATH + self.edge_marker.ns + '.json'):
            os.remove(CONFIG_PATH + self.edge_marker.ns + '.json')

    def save_edge(self):
        with open(CONFIG_PATH + self.edge_marker.ns + '.json', 'w+') as f:
            pose_dict = {"name": self.edge_marker.ns,
                         "start marker": self.int_mark_1.name,
                         "end marker": self.int_mark_2.name,
                         "pose": {
                             "start": {
                                "x": self.edge_marker.points[0].x,
                                "y": self.edge_marker.points[0].y,
                                "z": self.edge_marker.points[0].z
                             },
                             "end": {
                                "x": self.edge_marker.points[1].x,
                                "y": self.edge_marker.points[1].y,
                                "z": self.edge_marker.points[1].z,
                             },
                         },
                         }
            f.write(json.dumps(pose_dict, indent=4, sort_keys=True))
        rospy.loginfo('Successfully saved the Edge : {}'.format(self.edge_marker.ns))


if __name__ == '__main__':
    default_value = "/home/damien/pepper_ws/src/robocup-main/robocup_pepper-world_mng/map_manager/json/"
    rospy.init_node("pepper_interactive_marker")
    _cfgpath = rospy.get_param("~confPath", default_value)

    manager = MapMng(_cfgpath)
