import unittest
import time
import sys
import os
print(sys.path)
sys.path.append(os.path.dirname(__file__)+'/../')
print(sys.path)
from process.common.FifoEntity import FifoEntity

class Entity:
    def __init__(self,uuid):
        self.uuid=uuid
    def __str__(self):
     return self.uuid


class TestFifoEntity(unittest.TestCase):

    def test_creation(self):
        fifoEntity = FifoEntity(10, 10,5)
        self.assertTrue(True)

    def test_add_one_entity(self):
        fifoEntity = FifoEntity(10, 10,5)
        fifoEntity.add_entity(Entity("aaa"),1,2,3,"card")
        coord,entity=fifoEntity.getElt()
        print('coord:%s'%coord)
        print('entity:%s'%entity.uuid)
        self.assertEqual("aaa",entity.uuid)
        self.assertEqual("card",coord[3])

    def test_add_exceeded_entity(self):
        fifoEntity = FifoEntity(2, 10,5)
        for i in range(0,3):
            fifoEntity.add_entity(Entity("aaa"+str(i)),1,2,3,"card")
        coord,entity=fifoEntity.getElt()
        print('coord:%s'%coord)
        print('entity:%s'%entity.uuid)
        self.assertEqual("aaa1",entity.uuid)

    def test_timeout_entity(self):
        fifoEntity = FifoEntity(10, 10,2)
        for i in range(0,3):
            fifoEntity.add_entity(Entity("aaa"+str(i)),1,2,3,"card")
            time.sleep(1)
            print("Add Entity %i"%i)
        fifoEntity.check_old_elt_in_queue()
        coord,entity=fifoEntity.getElt()
        print('coord:%s'%coord)
        print('entity:%s'%entity.uuid)
        self.assertEqual("aaa2",entity.uuid)

    def test_get_info_list(self):
        fifoEntity = FifoEntity(10, 10,5)
        for i in range(0,8):
            fifoEntity.add_entity(Entity("aaa"+str(i)),1,2,3,"card")
            time.sleep(1)
            print("Add Entity %i"%i)
        fifoEntity.check_old_elt_in_queue()
        coordlist, entityList=fifoEntity.coord_fifo_to_list()
        for i in range(0,len(coordlist)):
            print(coordlist[i])
            print(entityList[i])
            self.assertEqual(coordlist[i][4],entityList[i].uuid)

if __name__ == '__main__':
    unittest.main()