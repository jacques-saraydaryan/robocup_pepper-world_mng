import unittest
import time
import sys
import os
print(sys.path)
sys.path.append(os.path.dirname(__file__)+'/../')
print(sys.path)
from process.EntityMergeMng import EntityMergeMng

class TestEntityMergeMng(unittest.TestCase):

    def test_creation(self):
        entity_mng = EntityMergeMng(10,10)
        self.assertTrue(True)

    def test_add_one_entity(self):
        entity_mng = EntityMergeMng(10,10)
        entity_mng.add_entity("blabla",5,5,5,"car")
        self.assertEqual(1,entity_mng.buffer_entity_list[entity_mng._current_buffer_indice].size())

    def test_add_entity_exceeded(self):
        entity_mng = EntityMergeMng(5,5)
        for i in range(0,6):
            try:
                entity_mng.add_entity("blabla"+str(i),i,5,5,"car")
            except Exception as e:
                print(e)
                self.assertEqual(5,i)
                return
        self.assertTrue(False)


    def test_process_buffer_3clusters(self):
        entity_mng = EntityMergeMng(10,5,dbscan_eps_value=1,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity("blabla1",0,0,5,"toto")
        entity_mng.add_entity("blabla2",0,0.5,5,"toto")
        entity_mng.add_entity("blabla3",0.5,0,5,"toto")
        entity_mng.add_entity("blabla4",0.5,0,4.3,"toto")

        entity_mng.add_entity("blabla5",3,2,0,"car")
        entity_mng.add_entity("blabla6",2.5,1.5,0,"car")
        entity_mng.add_entity("blabla7",3.2,1.75,0,"car")
        
        #cluster 3
        entity_mng.add_entity("blabla8",10,0.3,5,"bottle")
        entity_mng.add_entity("blabla9",9.7,0,5,"bottle")
        entity_mng.add_entity("blabla10",10.3,0.3,5,"bottle")

        clusters=entity_mng._process_buffer()
        self.assertEqual(3,len(clusters))

    def test_process_buffer_2clusters(self):
        entity_mng = EntityMergeMng(10,5,dbscan_eps_value=1.0,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity("blabla1",0,0,5,"toto")
        entity_mng.add_entity("blabla2",0,0.5,5,"toto")
        entity_mng.add_entity("blabla3",0.5,0,5,"toto")
        entity_mng.add_entity("blabla4",0.5,0,4.3,"toto")

        entity_mng.add_entity("blabla5",3,2,0,"car")
        entity_mng.add_entity("blabla6",2.5,1.5,0,"car")
        entity_mng.add_entity("blabla7",3.2,1.75,0,"boat")
        
        #cluster 3
        entity_mng.add_entity("blabla8",10,0.3,5,"bottle")
        entity_mng.add_entity("blabla9",9.7,0,5,"bottle")
        entity_mng.add_entity("blabla10",10.3,0.3,5,"bottle")

        clusters=entity_mng._process_buffer()
        self.assertEqual(2,len(clusters))

    def test_check_and_process_buffer_size_not_enought(self):
        entity_mng = EntityMergeMng(5,5,dbscan_eps_value=1.0,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity("blabla1",0,0,5,"car")
        entity_mng.add_entity("blabla2",0,0.5,5,"car")
        entity_mng.add_entity("blabla3",0.5,0,5,"car")
        entity_mng.add_entity("blabla4",0.5,0,4.3,"car")
        #entity_mng.add_entity("blabla5",3,5,5)

        clusters = entity_mng.check_and_process_buffer()
        self.assertEqual(0,len(clusters))

    def test_check_and_process_buffer_size_ok(self):
        entity_mng = EntityMergeMng(5,5,dbscan_eps_value=1.0,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity("blabla1",0,0,5,"car")
        entity_mng.add_entity("blabla2",0,0.5,5,"car")
        entity_mng.add_entity("blabla3",0.5,0,5,"car")
        entity_mng.add_entity("blabla4",0.5,0,4.3,"car")
        entity_mng.add_entity("blabla5",3,5,5,"car")

        clusters = entity_mng.check_and_process_buffer()
        self.assertEqual(1,len(clusters))

    def test_check_and_process_buffer_ttl_ok(self):
        entity_mng = EntityMergeMng(5,5,dbscan_eps_value=1.0,dbscan_min_samples=2)
        #cluster 1
        entity_mng.add_entity("blabla1",0,0,5,"car")
        entity_mng.add_entity("blabla2",0,0.5,5,"car")
        entity_mng.add_entity("blabla3",0.5,0,5,"car")
        #entity_mng.add_entity("blabla5",3,5,5)
        time.sleep(6)

        clusters = entity_mng.check_and_process_buffer()
        self.assertEqual(1,len(clusters))


if __name__ == '__main__':
    unittest.main()