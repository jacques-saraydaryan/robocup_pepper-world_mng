#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
import tf

from robocup_msgs.msg import Entity, Entity2D, EntityList
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Pose2D,Point
from process.EntityMergeMng import EntityMergeMng
from world_manager.srv import saveEntity_service
from convert_2d_to_3d.srv import SwitchMode,SwitchModeResponse

import time
import json
import Queue
from threading import Thread, Lock

class merge_register_data:
    PROCESS_MODE_REGISTRATION = 0
    PROCESS_MODE_GRASP = 1
    CLUSTER_BUILD_PERIOD = 5

    def __init__(self):
        self.configure_ros()
        self.cluster_to_publish = Queue.Queue()
        self.mergeProcessMng_lock = Lock()
        self._process_mode_lock = Lock()
        self._process_mode = self.PROCESS_MODE_REGISTRATION
        self._category_filter_tag_list=["*"]
        self._min_3d_frame_nb=1

        self.mergeProcessMngForRegistration = EntityMergeMng( self._buffer_size, 
                                               self._buffer_ttl,
                                               dbscan_eps_value=self._dbscan_eps_value,
                                               dbscan_min_samples = self._dbscan_min_samples)

        self.mergeProcessMngForGrasp = EntityMergeMng( self._grasp_buffer_size, 
                                               self._grasp_buffer_ttl,
                                               dbscan_eps_value=self._grasp_dbscan_eps_value,
                                               dbscan_min_samples = self._grasp_dbscan_min_samples)
        with self.mergeProcessMng_lock:
            self.mergeProcessMng = self.mergeProcessMngForRegistration
        

        self.publish_cluster_worker = Thread(target=self._check_and_process, args=())
        #self.publish_cluster_worker.setDaemon(True)
        self.publish_cluster_worker.start()
        
        self.check_and_process_worker = Thread(target=self._convert_and_publish, args=(1, self.cluster_to_publish,))
        #self.check_and_process_worker.setDaemon(True)
        self.check_and_process_worker.start()

        self.configure_ros_service_topic()


    def configure_ros(self):
        rospy.init_node('merge_register_data_node',anonymous=False)

        self._buffer_size = rospy.get_param('/convert_2d_to_3d/buffer_size', 1000)
        self._buffer_ttl = rospy.get_param('/convert_2d_to_3d/buffer_ttl', 10)
        self._dbscan_eps_value = rospy.get_param('/convert_2d_to_3d/dbscan_eps_value', 0.1)
        self._dbscan_min_samples = rospy.get_param('/convert_2d_to_3d/dbscan_min_samples', 2)

        self._grasp_buffer_size = rospy.get_param('/convert_2d_to_3d/grasp_buffer_size', 100)
        self._grasp_buffer_ttl = rospy.get_param('/convert_2d_to_3d/grasp_buffer_ttl', 2)
        self._grasp_dbscan_eps_value = rospy.get_param('/convert_2d_to_3d/grasp_dbscan_eps_value', 0.1)
        self._grasp_dbscan_min_samples = rospy.get_param('/convert_2d_to_3d/grasp_dbscan_min_samples', 1)

        self._db_radius_to_merge = rospy.get_param('/convert_2d_to_3d/db_radius_to_merge', 0.1)
        self.CLUSTER_BUILD_PERIOD = rospy.get_param('/convert_2d_to_3d/cluster_build_period', 5.0)

        rospy.loginfo('[merge_register_data_node] Parameters values:')
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/buffer_size: %i'%self._buffer_size)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/buffer_ttl: %f'%self._buffer_ttl)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/dbscan_eps_value: %f'%self._dbscan_eps_value)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/dbscan_min_samples: %i'%self._dbscan_min_samples)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/grasp_buffer_size: %i'%self._grasp_buffer_size)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/grasp_buffer_ttl: %f'%self._grasp_buffer_ttl)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/grasp_dbscan_eps_value: %f'%self._grasp_dbscan_eps_value)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/grasp_dbscan_min_samples: %f'%self._grasp_dbscan_min_samples)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/db_radius_to_merge: %f'%self._db_radius_to_merge)
        rospy.loginfo('[merge_register_data_node] - /convert_2d_to_3d/cluster_build_period: %f'%self.CLUSTER_BUILD_PERIOD)

        self._switch_service = rospy.Service('merge_register_data_switch_config', SwitchMode, self.configSwitcherServiceCallBack)


        
    def configure_ros_service_topic(self):
        try:
            rospy.wait_for_service('/save_Entity',5)
            rospy.loginfo("end service save_Entity wait time")
            #FIXME Change/create service message and service into world mng
            self._save_update_entity_service = rospy.ServiceProxy('save_Entity', saveEntity_service)
        #except rospy.ServiceException as e:
        except rospy.ROSException as e:
            rospy.logerr("Service save_Entity call failed: %s" % e)

        self.sub=rospy.Subscriber("/world_mng/objects/entity", EntityList, self.EntityCallback)

        #TOREMOVE CALL service instead
        #self.pub = rospy.Publisher('/world_mng/test_cluster', EntityList, queue_size=1)


    def EntityCallback(self,msg):
        entity_list = msg.entityList

        #process current entity (add to buffer)

        for entity in entity_list:
            #check if process is needed
            with self.mergeProcessMng_lock:
                self.mergeProcessMng.add_entity(entity,
                                            entity.pose.position.x,
                                            entity.pose.position.y,
                                            entity.pose.position.z,
                                            entity.label )

    def configSwitcherServiceCallBack(self,req):
        self._category_filter_tag_list=req.category_filter_tag_list
        with self._process_mode_lock:
            self._process_mode=req.register_or_grap_mode
    
        if(req.register_or_grap_mode == self.PROCESS_MODE_REGISTRATION):
            with self.mergeProcessMng_lock:
                self.mergeProcessMngForRegistration.reset_buffers()
                self.mergeProcessMng= self.mergeProcessMngForRegistration
        else:
            with self.mergeProcessMng_lock:
                self.mergeProcessMngForGrasp.reset_buffers()
                self.mergeProcessMng= self.mergeProcessMngForGrasp
        return SwitchModeResponse()
        

    def _check_and_process(self,):
        rospy.loginfo("[merge_register_data_node] THREAD - _check_and_process started")
        while not rospy.is_shutdown():    
            #rospy.loginfo("current buffer size: %i"%(mergeProcessMng.buffer_entity_list[self.mergeProcessMng._current_buffer_indice].size()))
            with self.mergeProcessMng_lock:
                t0 = time.time()
                clusters=self.mergeProcessMng.process_buffer()
                t1 = time.time()
                rospy.logwarn("[merge_register_data_node] --------------------------- Time to process fifo (making cluster nb:%i) t:%f, period to process: %f"%(len(clusters),t1-t0,self.CLUSTER_BUILD_PERIOD))
                
                if(len(clusters)!=0):
                    # put current clusters to process into the queue
                    # thread will process this cluster later
                    self.cluster_to_publish.put(clusters)
            rospy.sleep(self.CLUSTER_BUILD_PERIOD)
        rospy.loginfo("[merge_register_data_node] THREAD - _check_and_process STOPPED")

    def _convert_and_publish(self,i,clusters_queue):
        rospy.loginfo("[merge_register_data_node] THREAD - _convert_and_publish started")
        while not rospy.is_shutdown():
            if not clusters_queue.empty():
                clusters = clusters_queue.get(timeout=0.1)
                entity_list = EntityList()
                entity_list.header.stamp = rospy.get_rostime()
                for cluster_key in clusters:
                    current_cluster=clusters[cluster_key]
                    ref_entity=current_cluster['entity_list'][0]
                    ref_entity.header.stamp = rospy.get_rostime()
                    ref_entity.pose.position.x = current_cluster['c'][0]
                    ref_entity.pose.position.y = current_cluster['c'][1]
                    ref_entity.pose.position.z = current_cluster['c'][2]
                    if ref_entity.pose.orientation.x == 0 and ref_entity.pose.orientation.y==0 and ref_entity.pose.orientation.z==0 and ref_entity.pose.orientation.w ==0:
                        ref_entity.pose.orientation.x=0
                        ref_entity.pose.orientation.y=0
                        ref_entity.pose.orientation.z=0
                        ref_entity.pose.orientation.w=1
                    avg_confidence = 0
                    max_confidence = 0
                    score = len(current_cluster['entity_list'])
                    for entity in current_cluster['entity_list']:
                        try:
                            payload= json.loads(entity.payload)
                       
                            avg_confidence = avg_confidence + payload['confidence']
                            if max_confidence < payload['confidence']:
                                max_confidence = payload['confidence']
                        except Exception as e:
                            rospy.logwarn("pb with entity payload, %s"%(e))

                    avg_confidence =avg_confidence / len(current_cluster['entity_list'])
                    json_payload ={'score':score,'confidence':avg_confidence,'avg_confidence':avg_confidence,'max_confidence':max_confidence}
                    ref_entity.payload= json.dumps(json_payload)
                    entity_list.entityList.append(ref_entity)
                
                #FIXME TO REMOVE LOG
                #rospy.logwarn(entity_list)
                with self._process_mode_lock:           
                        if self._process_mode == self.PROCESS_MODE_REGISTRATION:
                            self._save_update_entity_service(entity_list,self._db_radius_to_merge)
                        else: 
                            self.broadcast_tf(entity_list)
                #self.pub.publish(entity_list)
            else:
                rospy.sleep(0.1)

        rospy.loginfo("[merge_register_data_node] THREAD - _convert_and_publish STOPPED")   
    
    def broadcast_tf(self,entity_list):
        """ Brocast TF of all entities
        - @param entity_list: EntityList message of the package robocup_msgs
        - Broacast a TF wit the followinf name format `Target_TF_i` (e.g Target_TF_0, Target_TF_1,etc..). i means the number of entity detected
        """
        br = tf.TransformBroadcaster()
        # get all object
        i=0
        for obj in entity_list.entityList:
            if obj.label in self._category_filter_tag_list or "*" in self._category_filter_tag_list:
                br.sendTransform((obj.pose.position.x, obj.pose.position.y,obj.pose.position.z),
                             (obj.pose.orientation.x, obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w),
                             rospy.Time(0),
                             'Target_TF_'+str(i),
                             "map")
            i=i+1


if __name__ == '__main__':
    m_r_data = merge_register_data()
    rospy.spin()