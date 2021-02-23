#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
import thread
from process.PostGisDao import PostGisDao
from rospy_message_converter import message_converter, json_message_converter
from robocup_msgs.msg import InterestPoint #, Order, OrderInterest
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from world_manager.srv import *
from tf import TransformListener
import tf
import time
import json



class WorldManagerNode:
    _node_is_ready = False
    _broadcastTfPeriod = 2
    _tfPublisherRunning = True

    def __init__(self,):
        self.configure_ros()
        self.configure()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    def configure(self):
        self.postgisDao=PostGisDao(host=self._postgres_ip,
                    port=self._postgres_port,
                    db_name=self._postgres_db_name,
                    user=self._postgres_user,
                    pwd=self._postgres_user_pwd,
                    re_create_db= self._re_create_db)
        self._node_is_ready=True

    def configure_ros(self):
        # Node configuration
        rospy.init_node('world_mng_node', anonymous=False)

        self._postgres_user = rospy.get_param('/postgres_user', "postgres")
        self._postgres_user_pwd= rospy.get_param('/postgres_user_pwd', "")
        self._postgres_db_name= rospy.get_param('/postgres_db_name', "world_mng_db")
        self._postgres_ip= rospy.get_param('/postgres_ip', "172.17.0.2")
        self._postgres_port= rospy.get_param('/postgres_port', "5432")
        self._re_create_db= rospy.get_param('/re_create_db', False)
        self._broadcastTfPeriod= rospy.get_param('/broadcast_tf_period', 2)
        rospy.loginfo("Param DB:  postgres_user:"+str( self._postgres_user))
        rospy.loginfo("Param DB:  postgres_db_name:"+str( self._postgres_db_name))
        rospy.loginfo("Param DB:  postgres_ip:"+str( self._postgres_ip))
        rospy.loginfo("Param DB:  postgres_port:"+str( self._postgres_port))
        rospy.loginfo("Param DB:  re_create_db:"+str( self._re_create_db))
        rospy.loginfo("Param DB:  broadcast_tf_period:"+str( self._broadcastTfPeriod))
        
        ##Define topics subscribe
        #self._update_object_sub=rospy.Subscriber("/world_mng/update_object", PoseStamped, self.updateObjectcallback)

        ##Define services
        self._loadPoint_service = rospy.Service('load_InterestPoint', Empty, self.loadInterestPointServiceCallback)
        self._savePoint_service = rospy.Service('save_InterestPoint', saveitP_service, self.saveInterestPointServiceCallback)
        #self._getPoint_service = rospy.Service('get_InterestPoint', getitP_service, self.getInterestPoint)
        #self._saveitPBaseLink_service = rospy.Service('save_BaseLinkInterestPoint', saveitP_base_link_service, self.saveBaseLinkInterestPoint)
        #self._activateTF_service = rospy.Service('activate_InterestPointTF', activateTF_service, self.activeTFProvider)

        self._tflistener = TransformListener()

        #self.loadInterestPoint()

        #start publishing It Tf
        thread.start_new_thread(self.publishInterestPointTf,())


    def updateObjectcallback(self, data):
        if not self._node_is_ready:
            return

    def publishInterestPointTf(self,):
        while(self._tfPublisherRunning and not rospy.is_shutdown()):
            if  self._node_is_ready:
                br = tf.TransformBroadcaster()
                # get all object
                obj_list = self.postgisDao.select_request("select * from object;")
                for obj in obj_list:
                    br.sendTransform((obj['x'], obj['y'], 0),
                                     (obj['orient_x'], obj['orient_y'],obj['orient_z'],obj['orient_w']),
                                     rospy.Time(0),
                                     str(obj['id'])+'_TF',
                                     "map")
            time.sleep(self._broadcastTfPeriod)

    def loadInterestPointServiceCallback(self,req):
        #TODO!!!
        rospy.logwarn('Nothing to do using POSTGIS DB.....')

    def saveInterestPointServiceCallback(self,req):
        if self._node_is_ready:
            data=req.itP
            json_payload = {"count":data.count,"overlap":data.overlap,"last_seen":data.last_seen,"confidence_darknet":data.confidence_darknet,"arm_position":data.arm_position,"head_pitch":data.head_pitch,"head_yaw":data.head_yaw}
            self.postgisDao.add_geo_object(data.label,'Object',
                                           data.pose.position.x,data.pose.position.y,data.pose.position.z,
                                           50,type_name=data.label,confidence=data.score,
                                           orient_x=data.pose.orientation.x,orient_y=data.pose.orientation.y,
                                           orient_z=data.pose.orientation.z,orient_w=data.pose.orientation.w,json_payload=json.dumps(json_payload))
            rospy.loginfo('Successfully saved the interestPoint:' + str(data))
        else:
            rospy.loginfo('Node world_mng_node is not ready, no save operation made')


        
        
        return True

def main():
    #""" main function
    #"""
    node = WorldManagerNode()

if __name__ == '__main__':
    main()