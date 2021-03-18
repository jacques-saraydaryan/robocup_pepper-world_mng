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
from threading import Thread

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
    THREAD_NB = 2
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
        for i in range(0,self.THREAD_NB):
            t = Thread(target=self._process_boxes, args=([i]))
            t.start()
            self._thread_list.append(t)
        


    def configure_ros(self):
        # Node configuration
        rospy.init_node('box_to_entity_node')

        self.filter = rospy.get_param('/convert_2d_to_3d/filter_class', "*")
        self.targetFrame = rospy.get_param('/convert_2d_to_3d/target_frame', "map")
        self.source_frame = rospy.get_param('/convert_2d_to_3d/source_frame', "kinect_depth_link")
        self.pixel_radius = rospy.get_param('/convert_2d_to_3d/pixel_radius', 10)

        
        self.bbox_time_threshold = rospy.get_param('/convert_2d_to_3d/bbox_time_threshold', 0.1)
        self.service_name = rospy.get_param('/convert_2d_to_3d/service_name', "convert_2d_to_3d")

        rospy.loginfo("Param :  filter_class:"+str(self.filter))
        rospy.loginfo("Param :  targetFrame:"+str(self.targetFrame))
        rospy.loginfo("Param :  source_frame:"+str(self.source_frame))
        rospy.loginfo("Param :  pixel_radius:"+str(self.pixel_radius))
        rospy.loginfo("Param :  bbox_time_threshold:"+str(self.bbox_time_threshold))
        rospy.loginfo("Param :  service_name:"+str(self.service_name))

        try:
            rospy.wait_for_service(self.service_name,5)
            rospy.loginfo("end service %s wait time"%(self.service_name))
            self._convert_2d_3d_service = rospy.ServiceProxy(self.service_name, get3Dfrom2D)
        except rospy.ServiceException as e:
            rospy.logerr("Service %s call failed: %s" % (self.service_name,e))

        self.sub=rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getDarkNetBoxesCallback)
        self.pub = rospy.Publisher('/world_mng/objects/entity', EntityList, queue_size=1)


    def _process_boxes(self, thread_name):
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
        now=rospy.get_rostime().secs
        msg_time_sec=msg.header.stamp.secs
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
                try:
                    current_point=self._convert_2d_3d_service(pose=current_pose2d,
                                                          frame_id=self.source_frame,
                                                          target_frame_id=self.targetFrame,
                                                          pixel_radius=self.pixel_radius,
                                                          stamp=current_stamp)
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
        
        try:
        
            self._box_to_process_fifo.put(msg, timeout=0.01)
        
        except Queue.Full as e:
        
            rospy.logwarn("Drop msg due to FIFO timeout")

        #self._process_boxe_msg(msg,'main')


if __name__ == '__main__':
    box2entity = box_to_entity()

    rospy.spin()