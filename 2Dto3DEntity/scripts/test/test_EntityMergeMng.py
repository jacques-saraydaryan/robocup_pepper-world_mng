import unittest
import time
import sys
import os
print(sys.path)
sys.path.append(os.path.dirname(__file__)+'/../')
print(sys.path)
from process.EntityMergeMng import EntityMergeMng
import rospy

class Entity:
    def __init__(self,uuid,label="fakeLabel"):
        self.uuid=uuid
        self.label=label

class TestEntityMergeMng(unittest.TestCase):

    def test_creation(self):
        entity_mng = EntityMergeMng(10,10)
        self.assertTrue(True)

    def test_add_one_entity(self):
        entity_mng = EntityMergeMng(10,10)
        entity_mng.add_entity(Entity("blabla"),5,5,5,"car")
        self.assertEqual(1,len(entity_mng._fifo_entity._fifo_entity.queue))

    def test_add_entity_exceeded(self):
        entity_mng = EntityMergeMng(5,5)
        for i in range(0,6):
            try:
                entity_mng.add_entity(Entity("blabla"+str(i)),i,5,5,"car")
            except Exception as e:
                print(e)
                self.assertTrue(True)
                return
        self.assertEqual("blabla1",entity_mng._fifo_entity._fifo_entity.get().uuid)


    def test_process_buffer_3clusters(self):
        entity_mng = EntityMergeMng(10,5,dbscan_eps_value=1,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity(Entity("blabla1"),0,0,5,"toto")
        entity_mng.add_entity(Entity("blabla2"),0,0.5,5,"toto")
        entity_mng.add_entity(Entity("blabla3"),0.5,0,5,"toto")
        entity_mng.add_entity(Entity("blabla4"),0.5,0,4.3,"toto")

        entity_mng.add_entity(Entity("blabla5"),3,2,0,"car")
        entity_mng.add_entity(Entity("blabla6"),2.5,1.5,0,"car")
        entity_mng.add_entity(Entity("blabla7"),3.2,1.75,0,"car")
        
        #cluster 3
        entity_mng.add_entity(Entity("blabla8"),10,0.3,5,"bottle")
        entity_mng.add_entity(Entity("blabla9"),9.7,0,5,"bottle")
        entity_mng.add_entity(Entity("blabla10"),10.3,0.3,5,"bottle")

        clusters=entity_mng.process_buffer()
        self.assertEqual(3,len(clusters))

    def test_process_buffer_2clusters(self):
        entity_mng = EntityMergeMng(10,5,dbscan_eps_value=1.0,dbscan_min_samples=3,display_process=False)
        #cluster 1
        entity_mng.add_entity(Entity("blabla1"),0,0,5,"toto")
        entity_mng.add_entity(Entity("blabla2"),0,0.5,5,"toto")
        entity_mng.add_entity(Entity("blabla3"),0.5,0,5,"toto")
        entity_mng.add_entity(Entity("blabla4"),0.5,0,4.3,"toto")

        entity_mng.add_entity(Entity("blabla5"),3,2,0,"car")
        entity_mng.add_entity(Entity("blabla6"),2.5,1.5,0,"car")
        entity_mng.add_entity(Entity("blabla7"),3.2,1.75,0,"boat")
        
        #cluster 3
        entity_mng.add_entity(Entity("blabla8"),10,0.3,5,"bottle")
        entity_mng.add_entity(Entity("blabla9"),9.7,0,5,"bottle")
        entity_mng.add_entity(Entity("blabla10"),10.3,0.3,5,"bottle")

        clusters=entity_mng.process_buffer()
        self.assertEqual(2,len(clusters))


    def test_check_and_process_buffer_size_ok(self):
        entity_mng = EntityMergeMng(5,5,dbscan_eps_value=1.0,dbscan_min_samples=3)
        #cluster 1
        entity_mng.add_entity(Entity("blabla1"),0,0,5,"car")
        entity_mng.add_entity(Entity("blabla2"),0,0.5,5,"car")
        entity_mng.add_entity(Entity("blabla3"),0.5,0,5,"car")
        entity_mng.add_entity(Entity("blabla4"),0.5,0,4.3,"car")
        entity_mng.add_entity(Entity("blabla5"),3,5,5,"car")

        clusters = entity_mng.process_buffer()
        self.assertEqual(1,len(clusters))

    #def test_check_and_process_buffer_ttl_ok(self):
    #    entity_mng = EntityMergeMng(5,5,dbscan_eps_value=1.0,dbscan_min_samples=2)
    #    #cluster 1
    #    entity_mng.add_entity(Entity("blabla1"),0,0,5,"car")
    #    entity_mng.add_entity(Entity("blabla2"),0,0.5,5,"car")
    #    entity_mng.add_entity(Entity("blabla3"),0.5,0,5,"car")
    #    #entity_mng.add_entity("blabla5",3,5,5)
    #    time.sleep(6)
#
    #    clusters = entity_mng.process_buffer()
    #    self.assertEqual(1,len(clusters))

    def test_check_clustering_with_large_different_categories(self):
        list=[['0.478925979158', '1.72041366053', '0.338162953405', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['0.839994215872', '1.39460499135', '0.220023868548', 'pitcher'], ['1.43720197384', '1.74155567275', '0.515741799004', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['3.10619888936', '3.18450739502', '0.000960778115468', 'coloredwoodblock'], ['3.13818368201', '3.49937216121', '0.000840265950091', 'coloredwoodblock'], ['1.83351236984', '5.08687828065', '0.861111093251', 'pottedmeat'], ['1.51749251286', '4.82177696414', '0.748751834135', 'sugar'], ['1.13746645355', '5.0060511129', '0.895427254161', 'pottedmeat'], ['0.858438474374', '4.62266843231', '0.961419217456', 'mustard'], ['0.691892589903', '4.95993932903', '0.862440519674', 'pottedmeat'], ['0.406098451559', '4.49834130458', '0.895938725193', 'mustard'], ['0.12812369009', '3.89425594954', '0.534817657272', 'legoduplo'], ['-0.27551997642', '4.73159381279', '0.600139205634', 'gelatin'], ['-0.583274241587', '4.63919945311', '0.600312986393', 'pottedmeat'], ['-0.725278029395', '4.54877949488', '0.530165151059', 'mustard'], ['-0.930979771043', '4.17856720669', '0.298943007401', 'lemon'], ['0.0', '0.0', '0.0', 'gelatin'], ['0.0', '0.0', '0.0', 'pottedmeat'], ['0.324886017928', '4.84101028613', '0.482104467322', 'tuna'], ['0.0', '0.0', '0.0', 'mustard'], ['-0.15773771022', '4.70698538593', '0.121250146812', 'strawberry'], ['-0.512578612166', '4.60848387188', '0.399463721501', 'lemon'], ['0.0', '0.0', '0.0', 'mustard'], ['0.113144736685', '4.78243481513', '0.233974019509', 'legoduplo'], ['-0.327620506584', '4.66011023016', '0.0769996761656', 'legoduplo'], ['0.438153607422', '4.8728203368', '0.347743386239', 'legoduplo'], ['-0.27024133607', '5.62588573594', '0.181816546111', 'legoduplo'], ['0.3229969383', '4.84099333885', '0.350572543268', 'legoduplo'], ['-0.655597595775', '3.44368888534', '0.390338001539', 'pitcher'], ['-0.240414552757', '5.08268149248', '0.311368182172', 'pitcher'], ['2.66803955614', '4.44257795974', '0.500825322086', 'legoduplo'], ['3.61838012352', '4.3264846375', '0.4755975006', 'cup'], ['3.62616664945', '3.72937549471', '0.000888229785055', 'legoduplo']]
        entity_mng = EntityMergeMng(100,100,dbscan_eps_value=0.5,dbscan_min_samples=2,display_process=True)
        i=0
        for elt in list:
            entity_mng.add_entity(Entity(elt[3]+str(i),label=elt[3]),elt[0],elt[1],elt[2],elt[3])
            i=i+1
        clusters = entity_mng.process_buffer()
        self.assertTrue(True)
    

if __name__ == '__main__':
    rospy.init_node('namo_scan', anonymous=True)
    unittest.main()