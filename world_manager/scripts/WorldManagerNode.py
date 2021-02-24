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
import datetime

class WorldManagerNode:
    _node_is_ready = False
    _broadcastTfPeriod = 2
    _tfPublisherRunning = True
    NO_TTL_VALUE =-1

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
        self._getPoint_service = rospy.Service('get_InterestPoint', getitP_service, self.getInterestPointServiceCallback)
        self._saveitPBaseLink_service = rospy.Service('save_BaseLinkInterestPoint', saveitP_base_link_service, self.saveBaseLinkInterestPointServiceCallback)
        self._activateTF_service = rospy.Service('activate_InterestPointTF', activateTF_service, self.activeTFProviderServiceCallback)

        self._tflistener = TransformListener()

        #start publishing It Tf
        thread.start_new_thread(self.publishInterestPointTf,())


    def updateObjectcallback(self, data):
        if not self._node_is_ready:
            return

    def publishInterestPointTf(self,):
        while( not rospy.is_shutdown()):
            if  self._node_is_ready and self._tfPublisherRunning:
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

    ######################
    ## ROS SERVICE CALL BACK ############################################
    ######################
    def getInterestPointServiceCallback(self,req):
        id = req.itP_name
        data = self.postgisDao.select_request("select * from object where id='%s'"%(id))
        if len(data)>0:
            current_data=data[0]
            current_payload= json.loads(current_data['json_payload'])

            itP=InterestPoint()
            itP.label = current_data['id']
            itP.score = current_data['confidence']
            itP.last_seen = self._datetime_to_float(current_data['update_date'])
            
            current_pose=Pose()
            current_pose.position.x = current_data['x']
            current_pose.position.y = current_data['y']
            current_pose.position.z = current_data['z']
            current_pose.orientation.x = current_data['orient_x']
            current_pose.orientation.y = current_data['orient_y']
            current_pose.orientation.z = current_data['orient_z']
            current_pose.orientation.w = current_data['orient_w']
            
            #FIXME need to differentiate InterestPOint and Entity2D/Entity3D
            if 'count' in current_payload:
                itP.count = current_payload['count']
            if 'overlap' in current_payload:
                itP.overlap = current_payload['overlap']
            if 'confidence_darknet' in current_payload:
                itP.confidence_darknet = current_payload['confidence_darknet']
            if 'arm_position' in current_payload:
                itP.arm_position = current_payload['arm_position']
            if 'head_pitch' in current_payload:
                itP.head_pitch = current_payload['head_pitch']
            if 'head_yaw' in current_payload:
                itP.head_yaw = current_payload['head_yaw']
            
            itP.pose=current_pose
            return itP

    def loadInterestPointServiceCallback(self,req):
        rospy.logwarn('Nothing to do using POSTGIS DB.....')

    def saveInterestPointServiceCallback(self,req):
        if self._node_is_ready:
            data=req.itP
            #FIXME need to differentiate InterestPOint and Entity2D/Entity3D
            json_payload = {"count":data.count,"overlap":data.overlap,"last_seen":data.last_seen,"confidence_darknet":data.confidence_darknet,"arm_position":data.arm_position,"head_pitch":data.head_pitch,"head_yaw":data.head_yaw}
            self.postgisDao.add_geo_object(data.label,'Object',
                                           data.pose.position.x,data.pose.position.y,data.pose.position.z,
                                           self.NO_TTL_VALUE,type_name=data.label,confidence=data.score,
                                           orient_x=data.pose.orientation.x,orient_y=data.pose.orientation.y,
                                           orient_z=data.pose.orientation.z,orient_w=data.pose.orientation.w,json_payload=json.dumps(json_payload))
            rospy.loginfo('Successfully saved the interestPoint:' + str(data))
        else:
            rospy.loginfo('Node world_mng_node is not ready, no save operation made')
        return True

    def saveBaseLinkInterestPointServiceCallback(self, req):
        now = rospy.Time(0)
        self._tflistener.waitForTransform("/map", "/base_link", now, rospy.Duration(10.0))
        (trans, rot) = self._tflistener.lookupTransform("/map", "/base_link", now)
        robotPose = Pose()
        robotPose.position.x = trans[0]
        robotPose.position.y = trans[1]
        robotPose.position.z = trans[2]
        robotPose.orientation.x = rot[0]
        robotPose.orientation.y = rot[1]
        robotPose.orientation.z = rot[2]
        robotPose.orientation.w = rot[3]       

        self.postgisDao.add_geo_object(req.label,'IT',
                                           robotPose.position.x,robotPose.position.y,robotPose.position.z,
                                           self.NO_TTL_VALUE,type_name=req.label,confidence=1,
                                           orient_x=robotPose.orientation.x,orient_y=robotPose.orientation.y,
                                           orient_z=robotPose.orientation.z,orient_w=robotPose.orientation.w)

        rospy.loginfo('Successful save ot the interestPoint' + str(req.label))
        return True

    def activeTFProviderServiceCallback(self, req):
        if(req.isActivated and self._tfPublisherRunning):
            rospy.loginfo('[MAP_MANAGER]: Keep tf broadcast enable')
            return []
        elif (not req.isActivated and self._tfPublisherRunning):
            self._tfPublisherRunning=False
            rospy.loginfo('[MAP_MANAGER]: Disable tf broadcast')
            return []
        elif ( req.isActivated and not self._tfPublisherRunning):
            self._tfPublisherRunning=True
            thread.start_new_thread(self.publishInterestPointTf,())
            rospy.loginfo('[MAP_MANAGER]: Start tf broadcast')
            return []
        if(not req.isActivated and not self._tfPublisherRunning):
            rospy.loginfo('[MAP_MANAGER]: Keep tf broadcast disable')
            return []
    
    ######################
    ## INTERNAL TOOLS ############################################
    ######################
    def _datetime_to_float(self,d):
        epoch = datetime.datetime.utcfromtimestamp(0)
        total_seconds =  (d - epoch).total_seconds()
        # total_seconds will be in decimals (millisecond precision)
        return total_seconds

    def _float_to_datetime(self,fl):
        return datetime.datetime.fromtimestamp(fl)

def main():
    #""" main function
    #"""
    node = WorldManagerNode()

if __name__ == '__main__':
    main()