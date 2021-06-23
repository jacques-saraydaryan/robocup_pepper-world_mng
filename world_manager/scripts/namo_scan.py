#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sklearn.cluster import DBSCAN
from sklearn import metrics
from geometry_msgs.msg import PointStamped,Pose
import tf
from copy import deepcopy
from world_manager.srv import getNamoEntity
import matplotlib.pyplot as plt

class NamoScan:
    OBSTACLE_VALUE = 100
    DBSCAN_EPS_VALUE= 0.1
    DBSCAN_EPS_VALUE_COMPUTED=1
    COSTMAP_RESOLUTION=0.025
    DBSAN_MIN_SAMPLES= 1
    DBSAN_NO_CLUSTER_LABEL = -1
    DISPLAY = False

    def __init__(self):
        rospy.init_node('namo_scan', anonymous=True)
        self.configure_ros()

    def configure_ros(self):
        self._listener = tf.TransformListener()
        self.COSTMAP_RESOLUTION= rospy.get_param('/move_base/local_costmap/resolution', 0.025)
        rospy.loginfo("Param COSTMAP_RESOLUTION:"+str( self.COSTMAP_RESOLUTION))
        #self._searchEntityInRoom_service = rospy.Service('search_Entity_in_room', select_object_in_room_service, self.searchEntityInRoomServiceCallback)
        self._local_costmap_sub=rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.CostmapCallback)
        rospy.Service("/get_namo_entity", getNamoEntity, self.ProcessObstacleCluster)

    def CostmapCallback(self, msg):
        self._costmap = msg
        number_of_cell=self.DBSCAN_EPS_VALUE/float(self.COSTMAP_RESOLUTION)


        self.DBSCAN_EPS_VALUE_COMPUTED= number_of_cell

    def coord_convert(self,x,y):
        """
        Given x, y.
        Return a PointStamped of the coord in /map
        """
        pose = PointStamped()
        pose.header.frame_id = "/base_footprint"
        pose.point.x = (x - 80) * self.COSTMAP_RESOLUTION
        pose.point.y = (-y + 80) * self.COSTMAP_RESOLUTION

        self._listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(2.0))
        point = self._listener.transformPoint("/map", pose)
        return point


    def isPtIntoBound(self, x1, y1, x2, y2, point):
        """
        Given bounding box x1, y1, x2, y2 and a point.
        Return True if the point is in the bounding box
        """
        if ((x1 - point.point.x) * (x2 - point.point.x)) <= 0 and ((y1 - point.point.y) * (y2 - point.point.y)) <= 0:
            return True
        return False
     

    def ProcessObstacleCluster(self, request):
        self.DBSCAN_EPS_VALUE = request.dbscan_eps
        self.OBSTACLE_VALUE = request.costmap_threshold_value
        self.DBSAN_MIN_SAMPLES = request.dbscan_min_sample
        #compute distance in nb of cell according given distance
        number_of_cell=self.DBSCAN_EPS_VALUE/float(self.COSTMAP_RESOLUTION)
        data = np.asarray(self._costmap.data, dtype=np.int8).reshape(self._costmap.info.height, self._costmap.info.width)

        data_xy, labels=self.process_cluster_obstacles(data, number_of_cell, self.DBSAN_MIN_SAMPLES, self.OBSTACLE_VALUE)


   
        clusters= self.compute_centroid(data_xy, labels)
        rospy.logdebug(clusters)
        resulted_clusted_centroid_points=namoScan.filter_clusters(clusters,request.x1,request.y1,request.x2,request.y2)
        return {"entity":resulted_clusted_centroid_points}

        

    def process_cluster_obstacles(self,costmap_matrix, dbscan_eps,dbscan_min_sample, costmap_threshold_value):
        unique, counts = np.unique(costmap_matrix, return_counts=True)
        print dict(zip(unique, counts))
       
        #print np.max(data)
        #print data
        costmap_matrix[costmap_matrix < 100] = 0

        unique, counts = np.unique(costmap_matrix, return_counts=True)
        #print dict(zip(unique, counts))
        #print data
        data_extracted = np. where(costmap_matrix == costmap_threshold_value)
        data_x=data_extracted[0]
        data_y=data_extracted[1]
        data_xy=np.column_stack((data_x,data_y))

        db = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_sample).fit(data_xy)
        
        #print (db)

        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        if self.DISPLAY:

            # Number of clusters in labels, ignoring noise if present.
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            
            # Black removed and is used for noise instead.
            unique_labels = set(labels)
            colors = [plt.cm.Spectral(each)
                    for each in np.linspace(0, 1, len(unique_labels))]
            for k, col in zip(unique_labels, colors):
                if k == -1:
                    # Black used for noise.
                    col = [0, 0, 0, 1]

                class_member_mask = (labels == k)

                xy = data_xy[class_member_mask & core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                        markeredgecolor='k', markersize=14)

                xy = data_xy[class_member_mask & ~core_samples_mask]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                        markeredgecolor='k', markersize=6)

            plt.title('Estimated number of clusters: %d' % n_clusters_)
            plt.show()

            #res = []
            #for e in data_xy:
            #    point = self.coord_convert(e[0], e[1])
            #    # if self.isPtIntoBound(2.36, 2.12, 3.18, 3.15, point):
            #    if self.isPtIntoBound(request.x1, request.y1, request.x2, request.y2, point):
            #        res.append(e)
            #res = np.array(res)
            #if self.DISPLAY:
            #    plt.plot(res[:, 0], res[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)
            #    plt.title('Filtered Clusters')
            #    plt.show()



        return data_xy, labels

            
        #return list of centroid

    def filter_clusters(self,data, x1, y1,x2, y2):
        #compute centroid


        res = []
        resulted_cluster_point=[]
        for e in data:
            point = self.coord_convert(data[e][0], data[e][1])
            # if self.isPtIntoBound(2.36, 2.12, 3.18, 3.15, point):
            if self.isPtIntoBound(x1, y1, x2, y2, point):
                res.append(data[e])
                resulted_cluster_point.append(self.pointToPos(point))
        #data = list(res.items())
        #es = np.array(res)
        
        res_array = np. array(res)
        print(res)
        print(res_array)
        return resulted_cluster_point

    def pointToPos(self,point):
        pose = Pose()
        pose.position.x=point.point.x
        pose.position.y=point.point.y
        pose.position.z=point.point.z
        pose.orientation.w=1
        return pose


        if self.DISPLAY:
            #plt.plot(res[:, 0], res[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)
            plt.plot(res_array[:, 0], res_array[:, 1], 'o', markeredgecolor='k', markersize=6)
            plt.title('Filtered Clusters')
            plt.show()
        #plt.imshow(data)
        #plt.colorbar()
        #plt.show()
        return resulted_cluster_point

    def compute_centroid(self,data,labels):
        # Collect coord info from cluster
        cluster_to_key={}
        for i in range(0,len(labels)):
            label_value=labels[i]
            if label_value != self.DBSAN_NO_CLUSTER_LABEL:
                if label_value not in cluster_to_key:
                    cluster_to_key[label_value]=[]
                cluster_to_key[label_value].append(i)

        # Create set of cluster with centroid of each cluster
        clusters={} # key = cluster label, entity_list = list of entity into cluser, c = centroid of clusters (x,y,z)
        for key_label in cluster_to_key:
            cluster_size = len(cluster_to_key[key_label])
            c_x=0
            c_y=0
            clusters[key_label]=[0,0]
            for coord_index in cluster_to_key[key_label]:
                c_x = c_x + data[coord_index][0]
                c_y = c_y + data[coord_index][1]
            avg_c_x=c_x/cluster_size
            avg_c_y=c_y/cluster_size
            clusters[key_label]=[avg_c_x,avg_c_y]
        return clusters


if __name__ == "__main__":

    namoScan = NamoScan()
    #rospy.sleep(1)
    #number_of_cell=namoScan.DBSCAN_EPS_VALUE/float(namoScan.COSTMAP_RESOLUTION)
    #costmap_matrix = np.asarray(namoScan._costmap.data, dtype=np.int8).reshape(namoScan._costmap.info.height, namoScan._costmap.info.width)
   #
    #data_xy, labels=namoScan.process_cluster_obstacles(costmap_matrix, number_of_cell,1, 100)
   #
    #clusters= namoScan.compute_centroid(data_xy, labels)
    #print(clusters)
    #res=namoScan.filter_clusters(clusters,0.32,-0.5,3.34,2.88)
    rospy.spin()
