import numpy as np
from common.BufferEntity import BufferEntity
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler,OneHotEncoder
from sklearn.compose import ColumnTransformer


class EntityMergeMng:
    #The maximum distance between two samples
    DBSCAN_EPS_VALUE = 0.3
    DBSAN_MIN_SAMPLES = 10
    DBSAN_NO_CLUSTER_LABEL = -1
    def __init__(self, buffer_size, ttl,dbscan_eps_value=0.3,dbscan_min_samples = 1):
        self._buffer_size = buffer_size
        self.DBSCAN_EPS_VALUE=dbscan_eps_value
        self.DBSAN_MIN_SAMPLES=dbscan_min_samples
        self.buffer_entity_list=[]
        self.buffer_entity_A = BufferEntity(self._buffer_size,ttl)
        self.buffer_entity_B = BufferEntity(self._buffer_size,ttl)
        self.buffer_entity_list.append(self.buffer_entity_A)
        self.buffer_entity_list.append(self.buffer_entity_B)
        self._current_buffer_indice=0

    def add_entity(self, entity, x, y ,z,category):
        if self.buffer_entity_list[self._current_buffer_indice].size() < self._buffer_size:
            self.buffer_entity_list[self._current_buffer_indice].add_entity(entity, x, y ,z,category)
        else:
           # self._switch_buffer()
           #self.buffer_entity_list[self._current_buffer_indice].add_entity(entity, x, y ,z)
           raise Exception('Buffer size exceed can not add entity, need to process buffer before')
        

    def check_and_process_buffer(self):
        if self.buffer_entity_list[self._current_buffer_indice].size() >= self._buffer_size or self.buffer_entity_list[self._current_buffer_indice].ttl() == 0:
            print("Process Buffer..... size[%i]"%(self.buffer_entity_list[self._current_buffer_indice].size())) 
            return self._process_buffer()
        # Else nothing to do 
        return {}
        
    def reset_buffers(self):
        for i in range (0, len(self.buffer_entity_list)):
            self.buffer_entity_list[i].reset()
        
        
    def _process_buffer(self):
        # Switch buffer to process data
        buffer_to_process = self.buffer_entity_list[self._current_buffer_indice] 

        if buffer_to_process.size() == 0:
            #reset current buffer, clear buffer, reset TTL
            buffer_to_process.reset()
            return {}

        # swtich to free buffer to collect new data
        self._switch_buffer()
        
        #get coord list from buffer
        coord_list= buffer_to_process.coord_buffer

        #use category
        #ask to modify only the column id 3 (label) as OnehotEncoder
        columnTransformer = ColumnTransformer([('encoder', OneHotEncoder(), [3])], remainder='passthrough') 
        #Apply modification on the given data
        data = np.array(columnTransformer.fit_transform(coord_list), dtype = np.float) 

        #Standardize features by removing the mean and scaling to unit variance
        #X = StandardScaler().fit_transform(coord_list)

        # Clusterized data using DBSCAN with x,y,z,category (one dimension per category --> OneHotEncoder)
        db = DBSCAN(eps=self.DBSCAN_EPS_VALUE, min_samples=self.DBSAN_MIN_SAMPLES).fit(data)

        # Collect coord info from cluster
        cluster_to_key={}
        for i in range(0,len(db.labels_)):
            label_value=db.labels_[i]
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
            c_z=0
            clusters[key_label]={'entity_list':[], 'c':(0,0,0)}
            for coord_index in cluster_to_key[key_label]:
                c_x = c_x + buffer_to_process.get_coord(coord_index)[0]
                c_y = c_y + buffer_to_process.get_coord(coord_index)[1]
                c_z = c_z + buffer_to_process.get_coord(coord_index)[2]
                clusters[key_label]['entity_list'].append(buffer_to_process.get_entity(coord_index))
            avg_c_x=c_x/cluster_size
            avg_c_y=c_y/cluster_size
            avg_c_z=c_z/cluster_size
            clusters[key_label]['c']=(avg_c_x,avg_c_y,avg_c_z)
        
        #reset current buffer, clear buffer, reset TTL
        buffer_to_process.reset()
        return clusters

    def build_clustered_entity(self, clusters):
        clustered_entity_list=[]
        for key in clusters:
            pass


    def _switch_buffer(self):
        next_indice=self._next_indice()
        if self.buffer_entity_list[next_indice].size()  == 0:
            self._current_buffer_indice = next_indice
        else:
            self.buffer_entity_list[next_indice].reset()
            raise Exception('Next Buffer is not empty, could not switch buffer buffer has been hard resetted')
        
    def _next_indice(self):
        if self._current_buffer_indice == 0:
            return 1
        else:
            return 0