#!/usr/bin/env python

# coding: utf-8

from PostGisDao import PostGisDao

if __name__ == '__main__':
    dao=PostGisDao(re_create_db=False)
    try:
        """Points d'interet ycb"""
        # dao.add_geo_object("Person","Itp",0.5,5.0,0,50,type_name="Person")
        # dao.add_geo_object("Kitchen","Itp",2.5,2.0,0,50,type_name="Kitchen")
        # dao.add_geo_object("Living_Room","Itp",3.4,6.3,0,50,type_name="LivingRoom",orient_x=0.0,orient_y=-0.0,orient_z=0.9999997,orient_w=0.0007963)
        # dao.add_geo_object("Bedroom","Itp",3.3,-2.3,0,50,type_name="Bedoom")
        # dao.add_geo_object("Tomato","Itp",0.45,-1.13,0,50,type_name="Entrance")
        # dao.add_geo_object("Pringles","Itp",-0.64,1.22,0,50,type_name="Entrance",orient_x=0.0,orient_y=-0.0,orient_z=0.7068252,orient_w=0.7073883)
        # dao.add_geo_object("Pringles_2","Itp",3.2851,2.46,0,50,type_name="Kitchen",orient_x=0.0,orient_y=-0.0,orient_z=0.7068252,orient_w=0.7073883)

        """Points d'interet RoboCup"""
        dao.add_geo_object("Room1","Itp",1.0,0.5,0,50,type_name="Itp")
        dao.add_geo_object("Room2","Itp",1.34,3.5,0,50,type_name="Itp")
        dao.add_geo_object("GreenBac","Itp",2.42,-0.6,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=-0.7068252,orient_w=0.7073883)
        dao.add_geo_object("Plate","Itp",1.78,-0.26,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=-0.7068252,orient_w=0.7073883)
        dao.add_geo_object("Perception_1","Itp",0.4,0.3,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.9238787,orient_w= 0.3826855)
        dao.add_geo_object("Perception_2","Itp",1.9,0.104,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.9226898,orient_w= 0.3855431)
        dao.add_geo_object("Perception_3_2","Itp",2.3,3.6,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.7068252,orient_w= 0.7073883)
        dao.add_geo_object("Perception_3_1","Itp",2.8,3.45,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.819152,orient_w= 0.5735764)
        dao.add_geo_object("Perception_3_3","Itp",1.65,3.45,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.3826834,orient_w= 0.9238796)
        dao.add_geo_object("Right_Person","Itp",0.82,4.0,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.9999997,orient_w= 0.0007963)
        dao.add_geo_object("Left_Person","Itp",0.82,2.9,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.9999997,orient_w= 0.0007963)
        dao.add_geo_object("NAMO_Observ","Itp",2.55,1.4,0,50,type_name="Itp",orient_x=0.0,orient_y=-0.0,orient_z=0.7071068,orient_w= 0.7071068)

    except Exception as err:
        print(err)