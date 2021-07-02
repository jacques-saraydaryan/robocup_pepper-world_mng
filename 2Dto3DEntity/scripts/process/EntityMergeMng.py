import numpy as np
from common.FifoEntity import FifoEntity
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler,OneHotEncoder
from sklearn.compose import ColumnTransformer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class EntityMergeMng:
    #The maximum distance between two samples
    DBSCAN_EPS_VALUE = 0.3
    DBSAN_MIN_SAMPLES = 10
    DBSAN_NO_CLUSTER_LABEL = -1
    DISPLAY_PROCESS = False
    def __init__(self, buffer_size, ttl,dbscan_eps_value=0.3,dbscan_min_samples = 1, display_process=False):
        self._buffer_size = buffer_size
        self.DBSCAN_EPS_VALUE=dbscan_eps_value
        self.DBSAN_MIN_SAMPLES=dbscan_min_samples
        self._fifo_entity=FifoEntity(buffer_size, ttl,ttl)
        self.DISPLAY_PROCESS=display_process

    def add_entity(self, entity, x, y ,z,category):
         self._fifo_entity.add_entity(entity, x, y ,z,category)
        

    def clean_old_entity_in_queue(self):
       self._fifo_entity.check_old_elt_in_queue()
        
    def reset_buffers(self):
        #for i in range (0, len(self.buffer_entity_list)):
        #    self.buffer_entity_list[i].reset()
        pass
        
    def process_buffer(self):
        #remove old value first
        self.clean_old_entity_in_queue()
        
        #get coord list from fifo
        coordlist, entityList=self._fifo_entity.coord_fifo_to_list()
        
        #crop coord values to fit clustering
        if len(coordlist) == 0:
                return {}
        m_coord = np.array(coordlist)
        m_coord_short= np.delete(m_coord, np.s_[4:6], axis=1)
        coordlist_cropped =  m_coord_short.tolist()
        print("----cropped list---")
       
        
        try:
            coordlist_cropped_array=np.array(coordlist_cropped)
            (unique, counts) = np.unique(coordlist_cropped_array[:,3], return_counts=True)
            print(unique)
            print(counts)
            print("|-----------------|")
        except Exception as e:
            print("Unexpected error:", e)

        #use category
        #ask to modify only the column id 3 (label) as OnehotEncoder
        try:
            columnTransformer = ColumnTransformer([('encoder', OneHotEncoder(), [3])], remainder='passthrough') 
            #Apply modification on the given data
            data = np.array(columnTransformer.fit_transform(coordlist_cropped), dtype = np.float) 
        #FIXME understood why such thing occurred
        # e.g of bad list : [['0.478925979158', '1.72041366053', '0.338162953405', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['0.839994215872', '1.39460499135', '0.220023868548', 'pitcher'], ['1.43720197384', '1.74155567275', '0.515741799004', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['3.10619888936', '3.18450739502', '0.000960778115468', 'coloredwoodblock'], ['3.13818368201', '3.49937216121', '0.000840265950091', 'coloredwoodblock'], ['1.83351236984', '5.08687828065', '0.861111093251', 'pottedmeat'], ['1.51749251286', '4.82177696414', '0.748751834135', 'sugar'], ['1.13746645355', '5.0060511129', '0.895427254161', 'pottedmeat'], ['0.858438474374', '4.62266843231', '0.961419217456', 'mustard'], ['0.691892589903', '4.95993932903', '0.862440519674', 'pottedmeat'], ['0.406098451559', '4.49834130458', '0.895938725193', 'mustard'], ['0.12812369009', '3.89425594954', '0.534817657272', 'legoduplo'], ['-0.27551997642', '4.73159381279', '0.600139205634', 'gelatin'], ['-0.583274241587', '4.63919945311', '0.600312986393', 'pottedmeat'], ['-0.725278029395', '4.54877949488', '0.530165151059', 'mustard'], ['-0.930979771043', '4.17856720669', '0.298943007401', 'lemon'], ['0.0', '0.0', '0.0', 'gelatin'], ['0.0', '0.0', '0.0', 'pottedmeat'], ['0.324886017928', '4.84101028613', '0.482104467322', 'tuna'], ['0.0', '0.0', '0.0', 'mustard'], ['-0.15773771022', '4.70698538593', '0.121250146812', 'strawberry'], ['-0.512578612166', '4.60848387188', '0.399463721501', 'lemon'], ['0.0', '0.0', '0.0', 'mustard'], ['0.113144736685', '4.78243481513', '0.233974019509', 'legoduplo'], ['-0.327620506584', '4.66011023016', '0.0769996761656', 'legoduplo'], ['0.438153607422', '4.8728203368', '0.347743386239', 'legoduplo'], ['-0.27024133607', '5.62588573594', '0.181816546111', 'legoduplo'], ['0.3229969383', '4.84099333885', '0.350572543268', 'legoduplo'], ['-0.655597595775', '3.44368888534', '0.390338001539', 'pitcher'], ['-0.240414552757', '5.08268149248', '0.311368182172', 'pitcher'], ['2.66803955614', '4.44257795974', '0.500825322086', 'legoduplo'], ['3.61838012352', '4.3264846375', '0.4755975006', 'cup'], ['3.62616664945', '3.72937549471', '0.000888229785055', 'legoduplo']]
        except ValueError as e:
            print('ERROR during converting list to numpy array and OneHotEncoder, may be to many categories, '+str(e))
            print('coordlist_cropped: ')
            print(coordlist_cropped)
            try:
                print("------RECOVERY MODE: Compute individual cluster for each category-----")
                current_cluster_key_index=0
                resulted_cluster={}
                data_array=np.array(coordlist_cropped)
                unique = np.unique(data_array[:,3], return_counts=False)
                for cat in unique:
                    print("------%s-----"%cat)
                    current_array_indices=np.where(data_array[:,3]==cat)
                    #FIXME complete and extract only value of the same label
                    #print(current_array_indices)
                    short_array=np.array(data_array[current_array_indices][:,:3],dtype=float)
                    #print(short_array)
                    current_cat_clusters=self.computeClusters(short_array, coordlist,entityList,index_converter_map=current_array_indices)
                    #fill the resulted cluster of each categories with all clusters detected on the given category
                    for key in current_cat_clusters:
                        resulted_cluster[current_cluster_key_index]=current_cat_clusters[key]
                        current_cluster_key_index+=1
                return resulted_cluster

            except Exception as e:
                print(e)
                self._fifo_entity.reset()
                return {}

        #Standardize features by removing the mean and scaling to unit variance
        #X = StandardScaler().fit_transform(coord_list)
        return self.computeClusters(data, coordlist,entityList)


    def computeClusters(self, data, coordlist,entityList, index_converter_map={}):
        # Clusterized data using DBSCAN with x,y,z,category (one dimension per category --> OneHotEncoder)
        try:
            db = DBSCAN(eps=self.DBSCAN_EPS_VALUE, min_samples=self.DBSAN_MIN_SAMPLES).fit(data)
        except Exception as e:
            print('ERROR during point clustering DBSCAN e:'+str(e))
            return {}
        #print("----------------------- Current DB ------------------")
        #print(db.labels_)

        if self.DISPLAY_PROCESS:
            #TODO display cluster
            #build xy matrix
            # matrix size
            matrix_elt_number = data.shape[1]
            data_x=data[:,matrix_elt_number-3]
            data_y=data[:,matrix_elt_number-2]
            data_z=data[:,matrix_elt_number-1]
            data_xyz=np.column_stack((data_x,data_y,data_z))

            core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
            core_samples_mask[db.core_sample_indices_] = True
            labels=db.labels_
            # Number of clusters in labels, ignoring noise if present.
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            #
            ## Black removed and is used for noise instead.
            unique_labels = set(labels)
            colors = [plt.cm.Spectral(each)
                    for each in np.linspace(0, 1, len(unique_labels))]
                    #check color in hexa for plot3d https://stackoverflow.com/questions/33596491/extract-matplotlib-colormap-in-hex-format
            colors_hexa=[]
            for c in colors:
                #convert to RBG format 255
                c_rgb=(int(round(c[0]*float(255))),int(round(c[1]*float(255))),int(round(c[2]*float(255))))
                #converto hexa
                current_hexa='#%02x%02x%02x' % c_rgb
                colors_hexa.append(current_hexa)

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            for k, col in zip(unique_labels, colors_hexa):
                if k == -1:
            #        # Black used for noise.
                    #col = [0, 0, 0, 1]
                    col='#000000'
                class_member_mask = (labels == k)
                xyz = data_xyz[class_member_mask & core_samples_mask]
                #plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                #        markeredgecolor='k', markersize=14)
                ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2],s=50, c=col, marker='^')

                xyz = data_xyz[class_member_mask & ~core_samples_mask]
                #plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                #        markeredgecolor='k', markersize=6)
                ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=col, marker='o')
            plt.title('Estimated number of clusters: %d' % n_clusters_)
            plt.show()

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
                if index_converter_map == {}:
                    c_x = c_x + float(coordlist[coord_index][0])
                    c_y = c_y + float(coordlist[coord_index][1])
                    c_z = c_z + float(coordlist[coord_index][2])
                    clusters[key_label]['entity_list'].append(entityList[coord_index])
                else:
                    remapped_index=index_converter_map[0][coord_index]
                    c_x = c_x + float(coordlist[remapped_index][0])
                    c_y = c_y + float(coordlist[remapped_index][1])
                    c_z = c_z + float(coordlist[remapped_index][2])
                    clusters[key_label]['entity_list'].append(entityList[remapped_index])

            avg_c_x=c_x/cluster_size
            avg_c_y=c_y/cluster_size
            avg_c_z=c_z/cluster_size
            clusters[key_label]['c']=(avg_c_x,avg_c_y,avg_c_z)


        #print("----------------------- Current clusters ------------------")
        #print(clusters)
        return clusters