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

class box_to_entity:
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
        self._box_to_process_fifo = Queue.Queue(self.THREAD_QUEUE_SIZE)
        # ThreadPool that execute the number of thread on the given tasks
        # here each Thread intends to get message into the fifo and process it
        self._thread_list=[]
        self._lock= Lock()
        for i in range(0,self.THREAD_NB):
            t = Thread(target=self._process_boxes, args=([i]))
            t.start()
            self._thread_list.append(t)
        


    def configure_ros(self):
        """ Configure ros node and all services, topics subscribers and publishers

        """
        # Node configuration
        rospy.init_node('box_to_entity_node')

        # Collect Ros parameters
        self.filter = rospy.get_param('/convert_2d_to_3d/filter_class', "*")
        self.targetFrame = rospy.get_param('/convert_2d_to_3d/target_frame', "map")
        self.source_frame = rospy.get_param('/convert_2d_to_3d/source_frame', "kinect_depth_link")
        self.pixel_radius = rospy.get_param('/convert_2d_to_3d/pixel_radius', 10)

        self.bbox_time_threshold = rospy.get_param('/convert_2d_to_3d/bbox_time_threshold', 0.1)
        self.service_name = rospy.get_param('/convert_2d_to_3d/service_name', "convert_2d_to_3d")

        box_frame_id_sources = rospy.get_param('/convert_2d_to_3d/box_frame_id_sources', ["kinect_depth_link"])
        box_2d_3d_services = rospy.get_param('/convert_2d_to_3d/box_2d_3d_services', ["convert_2d_to_3d"])
        self._frame_id_service_name_map={}

        #Display current ros parameters
        rospy.loginfo("Param :  filter_class:"+str(self.filter))
        rospy.loginfo("Param :  targetFrame:"+str(self.targetFrame))
        rospy.loginfo("Param :  source_frame:"+str(self.source_frame))
        rospy.loginfo("Param :  pixel_radius:"+str(self.pixel_radius))
        rospy.loginfo("Param :  bbox_time_threshold:"+str(self.bbox_time_threshold))
        rospy.loginfo("Param :  service_name:"+str(self.service_name))
        rospy.loginfo("Param :  box_frame_id_sources:"+str(box_frame_id_sources))
        rospy.loginfo("Param :  box_2d_3d_services:"+str(box_2d_3d_services))

        # if the number of frame_id and assiociated service name is different stop the current app.
        if len(box_frame_id_sources) != len(box_2d_3d_services):
            rospy.logerr("box_frame_id_sources and box_2d_3d_services sizes do not match")
            exit(0)

        # for all pair frame_id, service name try to create a ros service connection
        for i in range(0,len(box_frame_id_sources)):
            current_frame_id = box_frame_id_sources[i]
            current_service_name = box_2d_3d_services[i]
            try:
                rospy.wait_for_service(current_service_name,5)
                rospy.loginfo("end service %s wait time"%(current_service_name))
                current_convert_2d_3d_service = rospy.ServiceProxy(current_service_name, get3Dfrom2D)
                self._frame_id_service_name_map[current_frame_id]=current_convert_2d_3d_service
            except rospy.ServiceException as e:
                rospy.logerr("Service %s call failed: %s" % (current_service_name,e))
                self._frame_id_service_name_map[current_frame_id]=None
            except rospy.ROSException as e:
                rospy.logerr("Error:"+str(e))
                self._frame_id_service_name_map[current_frame_id]=None

        # Subscriber to the topic of object detection, frame_id of object could be different
        self.sub=rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getDarkNetBoxesCallback)
        self.pub = rospy.Publisher('/world_mng/objects/entity', EntityList, queue_size=1)


    def _process_boxes(self, thread_name):
        """ Internal process, Get a message from the queue and ask for processing

        """

        rospy.loginfo("[box_to_entity_node] Start thread %i -> _process_boxes"%(thread_name))
        while not rospy.is_shutdown():
            if not self._box_to_process_fifo.empty():
                try:
                    msg = self._box_to_process_fifo.get(timeout=self.THREAD_GET_TIMEOUT)
                    self._process_boxe_msg(msg,thread_name)
                except Queue.Empty as e:
                    #Timeout on fifo, no data to process (exeption allow rospy.is_shutdown detection), wait again
                    pass


    def _process_boxe_msg(self, msg, name):
        """ Internal message processing.
        The current fonction get message (bounding boxes) from the FIFO, for all objects:
        - get the bouding box center
        - ask the convert_2D_to_3D service (according to the frame_id) to get the avg 3D position of the center
        - create an Entity when 3D pt is available
        - publish the resulted Entity list

        - @Param name: current thread name
        - @Param msg: bounding boxe message to process
        """
        now=rospy.get_rostime().secs
        msg_time_sec=msg.header.stamp.secs
        #caution header.frame_id and .image_header.frame_id could be different..
        #current_frame_id=msg.header.frame_id
        current_frame_id=msg.image_header.frame_id
        diff=abs(now - msg_time_sec)
        if abs(now - msg_time_sec) > self.bbox_time_threshold:
            self._dropped_message_index=self._dropped_message_index+1
            self._count_dropped_message =self._count_dropped_message +1
            if self._dropped_message_index > self.LOG_DROPPED_MESSAGE_PERIOD:
                rospy.logwarn("Drop [%i] darknet message process[%s], too old >%fs, since %s"%(self.LOG_DROPPED_MESSAGE_PERIOD, name,self.bbox_time_threshold,str(self._dropped_message_last_log_time)))
                self._dropped_message_index =0
                self._dropped_message_last_log_time= str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            return
        entity_list=EntityList()
        list=[]
        for box in msg.bounding_boxes:
            if box.Class in self.filter or self.filter == self.DEFAULT_FILTER_ALL_VALUE:
                current_entity=Entity()
                current_entity.label=box.Class
                #rgb_w = int(( box.xmax-box.xmin)/2+box.xmin);
                #rgb_h = int((box.ymax-box.ymin)/2+box.ymin);
                center=[box.ymax-(box.ymax-box.ymin)/2,box.xmax-(box.xmax-box.xmin)/2]
                current_pose2d=Pose2D()
                current_pose2d.x=center[0]
                current_pose2d.y=center[1]
                #rospy.loginfo(box)
                #rospy.loginfo(current_pose2d)
                current_stamp =rospy.get_rostime()
                #lock ros service access object yo be used by multi-thread
                with self._lock:
                    try:
                        # get the service associated to the given frame_id
                        rospy.logdebug("Frame_id=[%s]"%(current_frame_id))
                        current_service = self._frame_id_service_name_map[current_frame_id]
                        if current_service != None:
                            #ask associated service to get the 3d position according the bouding box center, the source frame_id and the target frame_id
                            current_point=current_service(pose=current_pose2d,
                                                                  frame_id=current_frame_id,
                                                                  target_frame_id=self.targetFrame,
                                                                  pixel_radius=self.pixel_radius,
                                                                  stamp=current_stamp)
                        else:
                            rospy.logwarn("Service associated to frame_id :[%s] has not been created "%(current_frame_id))
                    except rospy.ServiceException as e:
                        self._svr_err_index=self._svr_err_index +1
                        if self._svr_err_index > self.SVR_ERR_MESSAGE_PERIOD:
                            self._svr_err_index = 0
                            rospy.logwarn("Service convert_2d_to_3d call failed %i times, unable to get conversion (no pt to process ??): %s" %(self.SVR_ERR_MESSAGE_PERIOD,e))
                        return
                current_entity.type = "Object"
                current_entity.pose.position.x=current_point.point.x
                current_entity.pose.position.y = current_point.point.y
                current_entity.pose.position.z = current_point.point.z
                current_entity.header.frame_id = self.targetFrame
                current_entity.header.stamp= current_stamp
                json_payload={}
                json_payload['confidence']=box.probability
                current_entity.payload=json.dumps(json_payload)
                list.append(current_entity)
        entity_list.entityList = list
        self.pub.publish(entity_list)


    def getDarkNetBoxesCallback(self,msg):
        """ Object Detection Callback
         Get the current message and put it into a FIFO to be processed later by a thread
         - @Param msg: bounding boxes list

        """
        try:
            self._box_to_process_fifo.put(msg, timeout=0.01)
        except Queue.Full as e:
            rospy.logwarn("Drop msg due to FIFO timeout")
        #self._process_boxe_msg(msg,'main')


if __name__ == '__main__':
    box2entity = box_to_entity()

    rospy.spin()