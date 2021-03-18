#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
from geometry_msgs.msg import PoseStamped
from world_manager.srv import saveEntity_service,saveEntity_serviceRequest
from robocup_msgs.msg import EntityList,Entity
import json
import random

class addEntityTest:

    def __init__(self):
        self.configure_ros()

        rospy.loginfo("Wait world Manager save_Entity service...")
        rospy.wait_for_service('save_Entity')
        rospy.loginfo("Wait world Manager save_Entity service FOUND !!")
        self._save_Entity_srv = rospy.ServiceProxy('save_Entity', saveEntity_service)

    def configure_ros(self):
         # Node configuration
        rospy.init_node('add_entity_test_node', anonymous=True)

    def addEntityList(self):
        entity_list = EntityList()
        entity_list.entityList=[]

        entity1 =Entity()

        entity1.uuid='1'
        entity1.label='chair'
        entity1.type='OBJECT'
        json_payload={'count':5,'confidence':0.3}
        entity1.payload=json.dumps(json_payload)
        entity1.pose.position.x=1
        entity1.pose.position.y=1
        entity1.pose.position.z=1
        entity1.pose.orientation.w=1

        entity_list.entityList.append(entity1)

        #req =saveEntity_serviceRequest()
        #req.radius=2
        #req.entity_list = entity_list
#
        #self._save_Entity_srv(req)

        self._save_Entity_srv(entity_list, 2)

    def addEntityListNear(self,nb):
        entity_list = EntityList()
        entity_list.entityList=[]
        for i in range (0,nb):    
            entity1 =Entity()

            entity1.uuid=str(1+ random.random())
            entity1.label='chair'
            entity1.type='OBJECT'
            json_payload={'count':1,'confidence':0.3}
            entity1.payload=json.dumps(json_payload)
            entity1.pose.position.x=1 + random.random()
            entity1.pose.position.y=1 + random.random()
            entity1.pose.position.z=1 + random.random()
            entity1.pose.orientation.w=1

            entity_list.entityList.append(entity1)

        self._save_Entity_srv(entity_list, 2)


    def addEntityListNearAndFar(self,nbN,nbF):
        entity_list = EntityList()
        entity_list.entityList=[]
        for i in range (0,nbN):    
            entity1 =Entity()

            entity1.uuid=str(1+ random.random())
            entity1.label='chair'
            entity1.type='OBJECT'
            json_payload={'count':1,'confidence':0.3}
            entity1.payload=json.dumps(json_payload)
            entity1.pose.position.x=1 + random.random()
            entity1.pose.position.y=1 + random.random()
            entity1.pose.position.z=1 + random.random()
            entity1.pose.orientation.w=1

            entity_list.entityList.append(entity1)

        for i in range (0,nbF):    
            entity1 =Entity()

            entity1.uuid=str(1+ random.random())
            entity1.label='chair'
            entity1.type='OBJECT'
            json_payload={'count':1,'confidence':0.1}
            entity1.payload=json.dumps(json_payload)
            entity1.pose.position.x=1 + random.random()*10
            entity1.pose.position.y=1 + random.random()*10
            entity1.pose.position.z=1 + random.random()*10
            entity1.pose.orientation.w=1

            entity_list.entityList.append(entity1)


        self._save_Entity_srv(entity_list, 2)

         



if __name__ == "__main__":
    add_test=addEntityTest()
    add_test.addEntityList()
    add_test.addEntityListNear(10)
    add_test.addEntityListNearAndFar(2,10)
    rospy.spin()
