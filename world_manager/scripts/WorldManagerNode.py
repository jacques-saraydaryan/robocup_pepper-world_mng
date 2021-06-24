#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
import thread
from process.PostGisDao import PostGisDao
from rospy_message_converter import message_converter, json_message_converter
from robocup_msgs.msg import InterestPoint, EntityList, Entity #, Order, OrderInterest
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from world_manager.srv import *
from tf import TransformListener
import tf
import time
import json
import datetime
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler,OneHotEncoder
from sklearn.compose import ColumnTransformer
import uuid

class WorldManagerNode:
    _node_is_ready = False
    _broadcastTfPeriod = 2
    _tfPublisherRunning = True
    NO_TTL_VALUE =-1
    DEFAULT_TTL_VALUE = 100

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

        self._postgres_user = rospy.get_param('/world_manager/postgres_user', "postgres")
        self._postgres_user_pwd= rospy.get_param('/world_manager/postgres_user_pwd', "")
        self._postgres_db_name= rospy.get_param('/world_manager/postgres_db_name', "world_mng_db")
        self._postgres_ip= rospy.get_param('/world_manager/postgres_ip', "172.17.0.2")
        self._postgres_port= rospy.get_param('/world_manager/postgres_port', "5432")
        self._re_create_db= rospy.get_param('/world_manager/re_create_db', False)
        self._broadcastTfPeriod= rospy.get_param('/world_manager/broadcast_tf_period', 2)
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
        self._saveEntity_service = rospy.Service('save_Entity', saveEntity_service, self.saveEntityServiceCallback)

        self._searchEntityInRoom_service = rospy.Service('search_Entity_in_room', select_object_in_room_service, self.searchEntityInRoomServiceCallback)
        self._searchEntityInRange_service = rospy.Service('search_Entity_in_range', select_object_in_range_service, self.searchEntityInRangeServiceCallback)

        
        self._tflistener = TransformListener()

        #start publishing It Tf
        thread.start_new_thread(self.publishInterestPointTf,())


    def updateObjectcallback(self, data):
        if not self._node_is_ready:
            return

    def publishInterestPointTf(self,):
        thread_name='tf_broacaster'
        while( not rospy.is_shutdown()):
            if  self._node_is_ready and self._tfPublisherRunning:
                br = tf.TransformBroadcaster()
                # get all object
                obj_list = []
                obj_list = self.postgisDao.select_request("select * from object;", thread_safe=True,thread_name=thread_name)
                for obj in obj_list:
                    br.sendTransform((obj['x'], obj['y'], obj['z']),
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

    def saveEntityServiceCallback (self, req):
        t0 = time.time()
        entity_map={}
        #gather set of clusters by category
        for entity in req.entity_list.entityList:
            if entity.label not in entity_map:

                entity_map[entity.label]=[]
            entity_map[entity.label].append(entity) 


        #For each entity of the same category
        # - detect clusters
        # - on each cluster call object in radius
        #   - Check if data in cluster needs to be merged to db data
        #   - Merge new data or create new entry if needed
        for key in entity_map:
            current_entity_list=[]
            current_coord_list=[]
            for entity in entity_map[key]:
                current_entity_list.append(entity)
                current_coord_list.append((entity.pose.position.x,entity.pose.position.y,entity.pose.position.z))
        
            self._add_or_update_object(current_entity_list,current_coord_list,key,req.radius)
            t1 = time.time()
            rospy.logdebug("--------------------------------- duration: %f, for saving %i entites"%(t1-t0,len(req.entity_list.entityList)))
        return True

    def searchEntityInRoomServiceCallback(self, req):
        room = req.room
        category_filter= req.category_filter
        confidence_filter = req.min_confidence_filter
        #Call the database with given parameters
        data_list = self.postgisDao.get_all_object_in_room(room)
        
        entityList = EntityList()
        entityList.entityList=[]
        for obj in data_list:
            #FIXME Add filter on last update
            if obj['confidence']>=confidence_filter:
                if obj['type_name'] in category_filter or '*' in category_filter:
                    current_entity=self._db_obj_to_entity(obj)
                    entityList.entityList.append(current_entity)
        return entityList

    def searchEntityInRangeServiceCallback(self, req):
        x= req.current_pose.position.x
        y= req.current_pose.position.y
        z= req.current_pose.position.z
        range_distance = req.range
        category_filter= req.category_filter
        confidence_filter = req.min_confidence_filter

        entityList = EntityList()
        entityList.entityList=[]

        if '*' in category_filter:
            #Call the database with given parameters
            #FIXME distance is computed only through x,y, results could be different
            #from get_obj_in_range_in_category_3D methods
            data_list = self.postgisDao.get_obj_in_range(x,y,range_distance)
            for obj in data_list:
                    #FIXME Add filter on last update
                    if obj['confidence']>=confidence_filter:
                        current_entity=self._db_obj_to_entity(obj)
                        entityList.entityList.append(current_entity)
        else:
           
            for category in category_filter:
                #Call the database with given parameters
                data_list_c = self.postgisDao.get_obj_in_range_in_category_3D(x,y,z,
                                                                     range_distance,category)
                for obj in data_list_c:
                    #FIXME Add filter on last update
                    if obj['confidence']>=confidence_filter:
                        current_entity=self._db_obj_to_entity(obj)
                        entityList.entityList.append(current_entity)
        return entityList
    
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

    def _db_obj_to_entity(self,db_obj):
        current_entity=Entity()
        current_pose=Pose()
        #{'id':id,'type':type,'x':x,'y':y,'ttl':ttl,'type_name':type_name,'confidence':confidence,'count':count,'orient_x':orient_x,'orient_y':orient_y,'orient_z':orient_z,'orient_w':orient_w,'update_date':update_date,'json_payload':json_payload}
        current_pose.position.x=db_obj['x']
        current_pose.position.y=db_obj['y']
        current_pose.position.z=db_obj['z']
        current_pose.orientation.x=db_obj['orient_x']
        current_pose.orientation.y=db_obj['orient_y']
        current_pose.orientation.z=db_obj['orient_z']
        current_pose.orientation.w=db_obj['orient_w']

        current_entity.pose=current_pose
        current_entity.uuid=db_obj['id']
        current_entity.type=db_obj['type_name']
        current_entity.type=db_obj['type_name']
        #diameter is not currently supported in DB
        current_entity.payload=db_obj['json_payload']
        return current_entity

    def _add_or_update_object(self, entity_list,coord_list,category,radius):
        """ Check if a set of object need to update existing data in Db or create new one
           - @param entity_list: list of entities to add to DB, CAUTIOn ASSUMING THAT ENTITY as attribute 'count' and 'confidence' into payload
           - @param coord_list: list of coordinate (x,y,z) of the given entities
           - @param category: category of the list of entities, needed to ask db of data of the same category
           - @param radius: radius in which entities will update existing data in db (use as eps in dbscan)
        """

        #Check if needed
        X = StandardScaler().fit_transform(coord_list)
        
        # Dbscan of entity same label eps 2 * radius, min =1
        # Clusterized data using DBSCAN with x,y,z
        db = DBSCAN(eps=2 * radius, min_samples=1).fit(X)
        cluster_index_list={}
        # gather all entity indices of a same label
        for i in range(0,len(db.labels_)):
            if db.labels_[i] not in cluster_index_list:
                cluster_index_list[db.labels_[i]]=[]
            cluster_index_list[db.labels_[i]].append(i)

        # compute for each cluster its centroid
        clusters={}
        for key_label in cluster_index_list:
            cluster_size = len(cluster_index_list[key_label])
            c_x=0
            c_y=0
            c_z=0
            clusters[key_label]={ 'c':(0,0,0),'max_dist_c':0}
            for coord_index in cluster_index_list[key_label]:
                c_x = c_x + coord_list[coord_index][0]
                c_y = c_y + coord_list[coord_index][1]
                c_z = c_z + coord_list[coord_index][2]
                #clusters[key_label]['entity_list'].append(buffer_to_process.get_entity(coord_index))
            avg_c_x=c_x/cluster_size
            avg_c_y=c_y/cluster_size
            avg_c_z=c_z/cluster_size

            clusters[key_label]['c']=(avg_c_x,avg_c_y,avg_c_z)

            #compute the max distance to the centroid
            for coord_index in cluster_index_list[key_label]:
                c = clusters[key_label]['c']
                dist_to_c = self.postgisDao._pt_distance(c[0],c[1],c[2],
                                              coord_list[coord_index][0],coord_list[coord_index][1],coord_list[coord_index][2])
                if clusters[key_label]['max_dist_c'] < dist_to_c:
                    clusters[key_label]['max_dist_c'] = dist_to_c
            
            # get all data in DB in range of cluster, radius_db =   max_dist_c + radius 
            c = clusters[key_label]['c']
            db_data_list = self.postgisDao.get_obj_in_range_in_category_3D(c[0],c[1],c[2],
                                                             clusters[key_label]['max_dist_c'] +radius,category)


            # if no object in DB in range add clusters to DB
            if len(db_data_list) == 0:
                for coord_index in cluster_index_list[key_label]:
                    entity_count = 1 
                    entity_confidence = 1
                    try:
                        entity_count = json.loads(entity_list[coord_index].payload)['count']
                        entity_confidence = json.loads(entity_list[coord_index].payload)['confidence']
                    except Exception as e:
                        print('Could not get count or confidence of entity to update DB, %s'%(e))
                    uuid_id = uuid.uuid1()
                    #Create new object
                    self.postgisDao.add_geo_object(uuid_id,entity_list[coord_index].type,
                                           coord_list[coord_index][0],coord_list[coord_index][1],coord_list[coord_index][2],
                                           1,
                                           entity_list[coord_index].label,entity_confidence,entity_count,
                                           json_payload=entity_list[coord_index].payload)
            else:

                for data in db_data_list:
                    current_db_confidence = data['confidence']
                    current_db_count = data['count']
                    current_db_x = data['x']
                    current_db_y = data['y']
                    current_db_z = data['z']
                    is_data_db_updated = False
                    min_dist_to_data_db = 500
                    min_dist_to_data_db_index=-1

                    # TODO need to lock an already detected object BD that already merge a sensor cluster
                    # Make Hungarian repartition with distance between Db objet and detect object as value
                    
                    #Find the closes cluster to the current data_db
                    for coord_index in cluster_index_list[key_label]:
                        current_distance =self.postgisDao._pt_distance(data['x'],data['y'],data['z'],
                                                            coord_list[coord_index][0],coord_list[coord_index][1],coord_list[coord_index][2] )
                        if min_dist_to_data_db > current_distance:
                            min_dist_to_data_db = current_distance
                            min_dist_to_data_db_index = coord_index

                    # If the min distance is < radius cluster as to be merged to data_db
                    if min_dist_to_data_db < radius:
                        is_data_db_updated = True

                    for coord_index in cluster_index_list[key_label]:
                        entity_count = 1 
                        entity_confidence = 1
                        try:
                            entity_count = json.loads(entity_list[coord_index].payload)['count']
                            entity_confidence = json.loads(entity_list[coord_index].payload)['confidence']
                        except Exception as e:
                            print('Could not get count or confidence of entity to update DB, %s'%(e))

                        if min_dist_to_data_db_index == coord_index and is_data_db_updated:
                            #Update current db object values
                            total_count = current_db_count + entity_count
                            current_db_x = (current_db_x * current_db_count + coord_list[coord_index][0] * entity_count ) /float(total_count)
                            current_db_y = (current_db_y * current_db_count + coord_list[coord_index][1] * entity_count ) /float(total_count)
                            current_db_z = (current_db_z * current_db_count + coord_list[coord_index][2] * entity_count ) /float(total_count)
                            current_db_confidence = (current_db_confidence * current_db_count + entity_confidence * entity_count) /float(total_count)
                            current_db_count = total_count
                        else:
                            uuid_id= uuid.uuid1()
                            #Create new object
                            self.postgisDao.add_geo_object(uuid_id,entity_list[coord_index].type,
                                               coord_list[coord_index][0],coord_list[coord_index][1],coord_list[coord_index][2],
                                               1,
                                               entity_list[coord_index].label,entity_confidence,entity_count,
                                               json_payload=entity_list[coord_index].payload)
                    if is_data_db_updated:
                        #Update existing object
                        self.postgisDao.add_geo_object(data['id'],data['type'],current_db_x,current_db_y,current_db_z,data['ttl'],
                                               data['type_name'],current_db_confidence,current_db_count,
                                               json_payload=data['json_payload'])

def main():
    #""" main function
    #"""
    node = WorldManagerNode()

if __name__ == '__main__':
    main()