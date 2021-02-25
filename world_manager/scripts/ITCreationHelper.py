#!/usr/bin/env python
__author__ = 'jsaraydaryan'
import rospy
from geometry_msgs.msg import PoseStamped
from world_manager.srv import saveitP_service
from robocup_msgs.msg import InterestPoint

class ITCreationHelper:
    _index_label = 0

    def __init__(self):
        self.configure_ros()

        rospy.loginfo("Wait world Manager save_InterestPoint service...")
        rospy.wait_for_service('save_InterestPoint')
        rospy.loginfo("Wait world Manager save_InterestPoint service FOUND !!")
        self._save_InterestPoint_srv = rospy.ServiceProxy('save_InterestPoint', saveitP_service)

    def configure_ros(self):
         # Node configuration
        rospy.init_node('it_creation_helper_node', anonymous=False)
        self._simpleGoal_sub=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simpleGoalcallback)

    def simpleGoalcallback(self, data):
        itPoint = InterestPoint()
        itPoint.label = "It"+str(self._index_label)
        self._index_label+=1
        itPoint.pose = data.pose
        itPoint.arm_position = 0

        try:
            result = self._save_InterestPoint_srv(itPoint)
            if result:
                rospy.loginfo("IT Point[%s] saved"%(itPoint.label))
            else:
                rospy.logwarn("IT Point[%s] NOT saved, a problem occured..."%(itPoint.label))

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    itHelper=ITCreationHelper()
    rospy.spin()
