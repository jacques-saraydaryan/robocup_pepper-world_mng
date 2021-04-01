import rospy
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import json
message = String(data = 'Hello')
json_str = json_message_converter.convert_ros_message_to_json(message)


class map_to_json():
    JSON_PATH="."
    JSON_FILE_NAME="map.json"

    def __init__(self):
        self.configure()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def configure(self):
        self._map_sub=rospy.Subscriber("/map", OccupancyGrid, self.mapcallback)

    def mapcallback(self,data):
        json_str = json_message_converter.convert_ros_message_to_json(data)
        #print(json_str)
        #json_obj=json.loads(json_str)
        with open(self.JSON_PATH+"/"+self.JSON_FILE_NAME, "w") as file:
            json.dump(json_str, file)
            rospy.loginfo("%s file successfully created, please close this node"%self.JSON_FILE_NAME)
        

if __name__ == '__main__':

    rospy.init_node('map_to_json')
    m = map_to_json()