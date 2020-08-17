
# map_manager


## 1. Description

The map_manager allows you to place interest points (referenced as Nodes or MapNodes) on the map and to make links (referenced as Edges or MapEdges) between them to help the robot navigate through the world or to represent positions that need a specific navigation strategy like doors or hallways.
Both are represented by [Rviz Interactive markers](http://wiki.ros.org/interactive_markers).
Once you placed all the nodes and edges you wanted on your map, the robot can use strategies based on those markers to have a simplified navigation, for that see the README of the [navigation_manager package](https://github.com/jacques-saraydaryan/ros_navigation_manager/tree/nav_mng_damien/navigation_manager)

Note: MapManager.py and MapTools.py are deprecated, you only need to launch map_mng.py to use Interactive markers -> see the sections below


## 2. Authors
* Jacques Saraydaryan
* Damien Jauneau


## 3. How to quote
F. Jumel, J. Saraydaryan, R. Leber, L. Matignon, E. Lombardi, C. Wolf and O. Simonin,”Context Aware Robot Architecture, Application to the Robocup@Home Challenge”, RoboCup Symposium 2018


## 4. Node
The name of the node is /pepper_map_manager

### 4.1 Subscribed Topics
### 4.1.1 Clicked point ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html))
Subscribe to the /clicked_point topic, published by Rviz when the user click on the map with the Publish point tool.
The pose is used to create an Interactive marker of type "MapNode".

### 4.2  Published Services
All published services have the namespace /pepper/

#### 4.2.1 add_node ([map_manager/AddNode](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/AddNode.srv))
Service to create a MapNode on the map through a [Node](https://github.com/jacques-saraydaryan/robocup_msgs/blob/dev_pepper/msg/Node.msg) message.
Return True if it succeeded and False otherwise.

### 4.2.2 get_node ([map_manager/GetNode](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/GetNode.srv))
Service to get the Node corresponding to the name in the request, the Node must have been loaded or it will not work.
Return the [MapNode](https://github.com/jacques-saraydaryan/robocup_msgs/blob/dev_pepper/msg/Node.msg) or None if it doesn't exist.

### 4.2.3 modify_node ([map_manager/ModifyNode](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/ModifyNode.srv))
Service to modify an attribute of a MapNode by giving its name, the attribute you want to change (for the moment just the colour) and the new value you give it.
Return True if it succeeded and False otherwise.

### 4.2.4 add_edge ([map_manager/AddEdge](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/AddEdge.srv))
Service to add a MapEdge through an [Edge](https://github.com/jacques-saraydaryan/robocup_msgs/blob/dev_pepper/msg/Edge.msg) message.

### 4.2.5 send_graph ([map_manager/SendGraph](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/SendGraph.srv))
Service to send a ["Graph"](https://github.com/jacques-saraydaryan/robocup_msgs/blob/dev_pepper/msg/Graph.msg) containing the list of the MapNodes and MapEdges currently used by the map_manager.

### 4.2.6 update_graph ([map_manager/UpdateGraph](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/UpdateGraph.srv))
Service to update the weights of edges linked to the node in the request. Let the "act" arg blank and put the name of the node and the weight you want to put on all its edges.
The "act" arg has only one value you can put for the moment : "reset" -> it will reset the weight of all the edges to 1

### 4.2.7 make_path ([map_manager/MakePath](https://github.com/jacques-saraydaryan/ros_world_mng/blob/dev_damien/map_manager/srv/MakePath.srv))
Service that makes a path of MapNodes between the two positions in args (start and end). The map is in two-dimensions, x and y, so the occupancy grid is an array of cells with costs between 0 (free) and 255 (occupied).
First it searches the nearest MapNode from the starting position and the one from the ending position using the "mode" arg :
"Mode" can have 3 values :
- euclidian : search the nearest MapNode by comparing the euclidian distance of all of them to the point, it doesn't take obstacles in account.
- ros_plan : same as euclidian but it uses the ROS make plan service (see below in the Subscribed services section) to calculate the distance, so it takes obstacles in account.
- fast_ros_plan : it makes one plan between the start and the end positions using the ROS make plan service, then it browse the list of points returned by the service and look for MapNodes in an area around the points (for now it's a square of 2m x 2m)

Once the two MapNodes have been found, it uses the djisktra algorithm on the Networkx graph to find the intermediary MapNodes.
Finally it sends a Graph message containing these MapNodes and the MapEdges that link them.

### 4.3 Subscribed Services
### 4.3.1 Static map ([nav_msgs/GetMap](http://docs.ros.org/melodic/api/nav_msgs/html/srv/GetMap.html))
Used to retrieve some information on the map currently used, like the frame_id and the resolution.

### 4.3.2 Make plan ([nav_msgs/GetPlan](http://docs.ros.org/melodic/api/nav_msgs/html/srv/GetPlan.html))
Calculate the shortest plan between two points without moving the robot. Used in some methods to search the nearest MapNode from a specific point.
 
### 4.4 Params
#### 4.4.1 confPath
Name of the config folder where interactive markers are loaded and saved, the default value takes the name of the map currently used by getting the value of the ROS param /map_file (this param is created in the [pepper_navigation.launch file](../../robocup_pepper-navigation_mng/pepper_nav_custom/launch/pepper_navigation.launch)) .
Otherwise it takes the name you give in parameter, like in the example below, when calling map_mng.py.
The folder is located in the [json directory](https://github.com/jacques-saraydaryan/ros_world_mng/tree/dev_damien/map_manager) under the map_manager package and if it doesn't exist (first time you use it or if you mispelled the name), the script will tell you and create it.

```
rosrun map_manager map_mng.py _confPath:="name_of_the_folder"
```

### 4.5 Interactive marker server
The map_mng.py script creates two [Interactive marker Server](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server).
One of name /pepper_nodes_server, for the management of the Interactive markers that represent the MapNodes (created when you use the Publish point tool of Rviz).
The other of name /pepper_edges_server, manages the Interactive markers that represent MapEdges, they are the links between MapNodes (created through the menu of a MapNode).


## 5. map_mng.py
### 5.1 Description
The map_mng.py allows you to load/create interest points on the map represented by Interactive markers with the Publish point tool of Rviz and then to make edges between with the interactive menu of the nodes.

### 5.2 How to use them
Interactive markers can be used without a map, on a empty Rviz configuration for example, but in our case we need them to represent points on a 2D map.
So once a roscore has been launched, start a map server, load a map and open Rviz like that :

```
rosrun map_server map_server "name_of_the_map".yaml     # You need to be in the directory of the map, or put the path before the name
```

```
rviz
```

Then launch the map_mng.py script with the _confPath arg if you want a specific name for your folder or not if you want the same name as the map (but it needs a name given by a ROS parameter named map_file)

```
rosrun map_manager map_mng.py (_confPath:="name_of_folder")
```

In the Displays panel, on the left, on Rviz, click Add then By topic and search the following ones :
 - Map under the /map topic
 - InteractiveMarkers under the /pepper_nodes_server/update topic
 - InteractiveMarkers under the /pepper_edges_server/update topic

Now that you can see the Interactive markers, select the Publish point tool on top of Rviz window and click on the map, a Node should appear :

![Rviz and node screenshot](rviz%20et%20node.png)

You can move it with the arrows or rotate it with the circle and right-click on it to open the menu :

![Node menu](menu%20node.png)

To create an Edge click on "create an edge" in the menu of one Node then do the same on an other Node, the edge should appear you can also right-click on it to open the menu :

![Edge menu](menu%20edge.png)

The "door" box is used to specify that the edge virtually crosses a door in the real environment. It's used for the GoThroughDoor strategy in the navigation_manager.

All your Nodes and Edges are saved in the folder you chosed at the beginning and you can find them in the json directory.

