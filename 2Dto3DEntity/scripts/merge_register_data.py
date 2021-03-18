#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy

from robocup_msgs.msg import Entity, Entity2D, EntityList
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Pose2D,Point
from process.EntityMergeMng import EntityMergeMng
from world_manager.srv import saveEntity_service

import json
import Queue
from threading import Thread

class merge_register_data:

    def __init__(self):
        self.configure_ros()
        self.cluster_to_publish = Queue.Queue()
        self.mergeProcessMng = EntityMergeMng( self._buffer_size, 
                                               self._buffer_ttl,
                                               dbscan_eps_value=self._dbscan_eps_value,
                                               dbscan_min_samples = self._dbscan_min_samples)
        self.publish_cluster_worker = Thread(target=self._check_and_process, args=())
        #self.publish_cluster_worker.setDaemon(True)
        self.publish_cluster_worker.start()
        
        self.check_and_process_worker = Thread(target=self._convert_and_publish, args=(1, self.cluster_to_publish,))
        #self.check_and_process_worker.setDaemon(True)
        self.check_and_process_worker.start()

        self.configure_ros_service_topic()

    def configure_ros(self):
        rospy.init_node('merge_register_data_node',anonymous=False)

        self._buffer_size = rospy.get_param('/2d_to_3d_entity/buffer_size', 1000)
        self._buffer_ttl = rospy.get_param('/2d_to_3d_entity/buffer_ttl', 10)
        self._dbscan_eps_value = rospy.get_param('/2d_to_3d_entity/dbscan_eps_value', 0.1)
        self._dbscan_min_samples = rospy.get_param('/2d_to_3d_entity/dbscan_min_samples', 2)
        self._db_radius_to_merge = rospy.get_param('/2d_to_3d_entity/db_radius_to_merge', 0.1)
        
    def configure_ros_service_topic(self):
        try:
            rospy.wait_for_service('/save_Entity',5)
            rospy.loginfo("end service save_Entity wait time")
            #FIXME Change/create service message and service into world mng
            self._save_update_entity_service = rospy.ServiceProxy('save_Entity', saveEntity_service)
        except rospy.ServiceException as e:
            rospy.logerr("Service save_Entity call failed: %s" % e)

        self.sub=rospy.Subscriber("/world_mng/objects/entity", EntityList, self.EntityCallback)
        #TOREMOVE CALL service instead
        #self.pub = rospy.Publisher('/world_mng/test_cluster', EntityList, queue_size=1)


    def EntityCallback(self,msg):
        entity_list = msg.entityList

        #process current entity (add to buffer)
        for entity in entity_list:
            #check if process is needed
            clusters=self.mergeProcessMng.check_and_process_buffer()
            if(len(clusters)!=0):
                # put current clusters to process into the queue
                # thread will process this cluster later
                self.cluster_to_publish.put(clusters)
            
            self.mergeProcessMng.add_entity(entity,
                                            entity.pose.position.x,
                                            entity.pose.position.y,
                                            entity.pose.position.z,
                                            entity.label )

    def _check_and_process(self,):
        rospy.loginfo("[merge_register_data_node] THREAD - _check_and_process started")
        while not rospy.is_shutdown():    
            #rospy.loginfo("current buffer size: %i"%(self.mergeProcessMng.buffer_entity_list[self.mergeProcessMng._current_buffer_indice].size()))
            clusters=self.mergeProcessMng.check_and_process_buffer()
            if(len(clusters)!=0):
                # put current clusters to process into the queue
                # thread will process this cluster later
                self.cluster_to_publish.put(clusters)
            rospy.sleep(0.1)
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
                #rospy.loginfo(entity_list)
                self._save_update_entity_service(entity_list,self._db_radius_to_merge)
                #self.pub.publish(entity_list)
            else:
                rospy.sleep(0.1)

        rospy.loginfo("[merge_register_data_node] THREAD - _convert_and_publish STOPPED")   

if __name__ == '__main__':
    m_r_data = merge_register_data()
    rospy.spin()