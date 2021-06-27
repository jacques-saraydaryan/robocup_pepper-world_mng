#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy

from convert_2d_to_3d.srv import get3Dfrom2D
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes
from robocup_msgs.msg import Entity, Entity2D, EntityList
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Pose2D,Point
import json
import datetime
import Queue

from concurrent.futures import ThreadPoolExecutor,ProcessPoolExecutor
from threading import Thread, Lock

class box_multiplexor:
    #DEFAULT_FILTER_VALUE="person"
    DEFAULT_FILTER_VALUE="*"
    DEFAULT_FILTER_ALL_VALUE="*"
    #DEFAULT_SOURCE_FRAME_VALUE="CameraTop_optical_frame"
    DEFAULT_SOURCE_FRAME_VALUE="kinect_depth_link"
    DEFAULT_TARGET_FRAME_VALUE = "map"
    DEFAULT_PIXEL_RADIUS_VALUE = 10
    _count_dropped_message=0
    _dropped_message_index=0
    _dropped_message_last_log_time = 0
    LOG_DROPPED_MESSAGE_PERIOD = 100
    _svr_err_index=0
    SVR_ERR_MESSAGE_PERIOD = 100
    THREAD_NB = 5
    THREAD_QUEUE_SIZE = 100
    THREAD_GET_TIMEOUT = 0.1

    def __init__(self):
        self.configure_ros()
        self._count_dropped_message =0
        #create a fifo to stock messages


    def configure_ros(self):
        """ Configure ros node and all services, topics subscribers and publishers

        """
        # Node configuration
        rospy.init_node('box_multiplexor')

        # Collect Ros parameters
        self.filter = rospy.get_param('/convert_2d_to_3d/filter_class', "*")
        self.bbox_time_threshold = rospy.get_param('/convert_2d_to_3d/bbox_time_threshold', 0.1)

        box_frame_id_sources = rospy.get_param('/convert_2d_to_3d/box_frame_id_sources', ["kinect_depth_link"])
        box_2d_3d_topics = rospy.get_param('/convert_2d_to_3d/box_2d_3d_topics', ["/darknet_ros/bounding_boxes_arm"])

        #Display current ros parameters
        rospy.loginfo("Param :  filter_class:"+str(self.filter))
        rospy.loginfo("Param :  bbox_time_threshold:"+str(self.bbox_time_threshold))
        rospy.loginfo("Param :  box_frame_id_sources:"+str(box_frame_id_sources))
        rospy.loginfo("Param :  box_2d_3d_topics:"+str(box_2d_3d_topics))

        # if the number of frame_id and assiociated service name is different stop the current app.
        if len(box_frame_id_sources) != len(box_2d_3d_topics):
            rospy.logerr("box_frame_id_sources and box_2d_3d_topics sizes do not match")
            exit(0)

        # for all pair frame_id, topic name, create a ros subscriber
        self._frame_id_topic_name_map={}
        self._frame_id_topic_name_name_map={}
        for i in range(0,len(box_frame_id_sources)):
            current_frame_id = box_frame_id_sources[i]
            current_topic_name = box_2d_3d_topics[i]
            current_pub = rospy.Publisher(current_topic_name, BoundingBoxes, queue_size=1)    
            self._frame_id_topic_name_map[current_frame_id]=current_pub
            self._frame_id_topic_name_name_map[current_frame_id]=current_topic_name

        # Subscriber to the topic of object detection, frame_id of object could be different
        self.sub=rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getDarkNetBoxesCallback)



    def getDarkNetBoxesCallback(self,msg):
        """ Object Detection Callback
         Get the current message and send it to the appropriate conver2D_to_3D topics
         - @Param msg: bounding boxes list

        """
        try:
            current_frame_id=msg.image_header.frame_id
            self._frame_id_topic_name_map[current_frame_id].publish(msg)
            #print('send msg frame_id:%s to topics:%s'%(current_frame_id,str(self._frame_id_topic_name_name_map[current_frame_id])))
        except KeyError as e:
            rospy.logwarn("[]")




if __name__ == '__main__':
    box_multi = box_multiplexor()

    rospy.spin()