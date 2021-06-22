import time
import Queue
import numpy as np
import threading


class FifoEntity:

    def __init__(self, fifo_size, buffer_ttl,data_ttl):
        self._lock= threading.Lock()
        self._fifo_coord = Queue.Queue(fifo_size)
        #To be improved by heapdict ??
        self._fifo_entity = Queue.Queue(fifo_size)
        self.fifo_size = fifo_size
        self.buffer_ttl = buffer_ttl
        self.data_ttl = data_ttl
        self.reset()

    def add_entity(self,entity,x,y,z,category):
        coord=[]
        coord.append(x)
        coord.append(y)
        coord.append(z)
        coord.append(category)
        coord.append(entity.uuid)
        coord.append(time.time())
        if self._fifo_coord.full():
            #if size exceeded remove oldest object and add new one
            with  self._lock:
                obj_removed= self._fifo_coord.get()
                print("object removed (too many obj in fifo): %s"%obj_removed)
                #CAUTION need to add mutex of _fifo_entity into previous block ??
                obj_removed= self._fifo_entity.get()
                print("object removed (too many obj in fifo): %s"%obj_removed.uuid)
        # add elt to the fifo
        #with self._fifo_coord.mutex:
        self._fifo_coord.put_nowait(coord)
        #with self._fifo_entity.mutex:   
        self._fifo_entity.put_nowait(entity)
            #self.entity_buffer[entity.uuid]=entity
            #self._last_update_time = seconds = time.time()
            #raise Exception('BufferEntity size exceed')

    def getElt(self):
        coord=[]
        entity={}
        with  self._lock:
            coord=self._fifo_coord.get()
            entity=self._fifo_entity.get()
        return coord,entity

    def reset(self):
        # key: coord_buffer position, value : the entity
        self.entity_buffer={}
        with  self._lock:
            self._fifo_coord.queue.clear()
            self._fifo_entity.queue.clear()
        self._last_update_time = seconds = time.time() # time in sec since epoch

    def coord_fifo_to_list(self):
        coord_list=[]
        entity_list=[]
        with  self._lock:
            coord_list=list(self._fifo_coord.queue)
            #m_coord=np.array(coord_list)
            #m_coord_short_= np.delete(m_coord, np.s_[4:6], axis=1)
            entity_list=list(self._fifo_entity.queue)
        return coord_list,entity_list

    def size(self):
        with  self._lock:
            return self._fifo_coord.qsize()

    def ttl(self):
        diff_time = time.time()-self._last_update_time
        if diff_time > self.buffer_ttl:
            return 0
        else:
            return diff_time

    def check_old_elt_in_queue(self):
        index_to_remove=[]
        current_time=time.time()
        with  self._lock:
            #for i in range(self._fifo_coord.qsize()-1,0,-1):
            for i in range(0,self._fifo_coord.qsize()):
                obj_time=self._fifo_coord.queue[i][5]
                diff_time = current_time-obj_time
                #print(self._fifo_coord.queue)
                print('queue index:%i, entity uuid:%s, diff time %f, time %f'%(i,self._fifo_coord.queue[i][4],diff_time,self._fifo_coord.queue[i][5]))
                if diff_time > self.data_ttl:
                    index_to_remove.append(i)
                else:
                    #no elt could be > self.data_ttl, data are order in time
                    break
        with  self._lock:
            for i in range(0,len(index_to_remove)):
                obj_removed= self._fifo_coord.get()
                #print("object removed (too old): %s"%obj_removed)
            for i in range(0,len(index_to_remove)):
                obj_removed= self._fifo_entity.get()
                print("object removed (too old): %s"%obj_removed.label)


    #def display_content(self):
    #    # to do iterate on entity_buffer sum the label and display sumup
    #    log_count_label_map={}
    #    for key in self.entity_buffer:
    #        current_entity=self.entity_buffer[key]
    #        if current_entity.label in log_count_label_map:
    #            log_count_label_map[current_entity.label] +=1
    #        else:
    #            log_count_label_map[current_entity.label] = 1
    #    return log_count_label_map