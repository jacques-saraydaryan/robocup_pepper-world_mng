import time
class BufferEntity:

    def __init__(self, buffer_size, buffer_ttl):
        self.buffer_size = buffer_size
        self.buffer_ttl = buffer_ttl
        
        self.reset()

    def add_entity(self,entity,x,y,z,category):
        if len(self.entity_buffer)< self.buffer_size:
            coord=[]
            coord.append(x)
            coord.append(y)
            coord.append(z)
            coord.append(category)

            new_id = len(self.coord_buffer)
            self.coord_buffer.append(coord)
            self.entity_buffer[new_id]=entity
            #self._last_update_time = seconds = time.time()
        else:
             raise Exception('BufferEntity size exceed')

    def reset(self):
        # key: coord_buffer position, value : the entity
        self.entity_buffer={}
        self.coord_buffer=[]
        self._last_update_time = seconds = time.time() # time in sec since epoch


    def get_entity(self,key):
        return self.entity_buffer[key]

    def get_coord(self,key):
        return self.coord_buffer[key]

    def size(self):
        return len(self.entity_buffer)

    def ttl(self):
        diff_time = time.time()-self._last_update_time
        if diff_time > self.buffer_ttl:
            return 0
        else:
            return diff_time