/*
 * 
 *      Author: jsaraydaryan
 */

#include <ros/ros.h>
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "robocup_msgs/Entity.h"
#include "robocup_msgs/EntityList.h"
#include "robocup_msgs/Entity2D.h"
#include "sensor_msgs/PointCloud2.h"
#include "convert_2d_to_3d/get3Dfrom2D.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h> 
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <cstdlib>
#include <string>
#include <exception>

// Publisher used to send evidence to world model
ros::Publisher pub_3D;
ros::Publisher object_marker_pub,object_entity_pub;
ros::Subscriber sub_registered_darknetBoxes;
ros::Subscriber sub_registered_pcl;
//tf::TransformListener listener;

sensor_msgs::PointCloud2 current_pcl;

pcl::PointCloud<pcl::PointXYZ> point_pcl;
sensor_msgs::PointCloud2::ConstPtr current_cloud;

int height;
std::string FRAME_ID="";
std::string filter;
std::string target_frame,source_frame,pcl_topic;
std::string service_name;
int measure_radius;

bool display_marker=true;
bool lock=false;
bool enable_lock_pcl=false;
double time_diff_threshold;
/*
* CAN BE TESTED WITH
* rosservice call /convert_2d_to_3d "{'pose':{'x':10,'y':10}}"
* rosservice call /convert_2d_to_3d "{'pose':{'x':11,'y':11},'pixel_radius':20}"
* rosservice call /convert_2d_to_3d "{'pose':{'x':0,'y':0},'pixel_radius':0,'frame_id':'/palbator_arm_kinect_link', 'target_frame_id':'/map'}"
*/




void getPclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
try{
	if(!lock || !enable_lock_pcl){
	//ROS_INFO("--> inside getPclCallback");
	//int pcl_index, rgb_h, rgb_w;
	//rgb_h = 240;
	//rgb_w = 320;
	height=cloud->height;
	//pcl_index = ( (rgb_w*cloud->height) + rgb_h);


	current_cloud=cloud;
	//std::cout << "(x,y,z) = " << point_pcl.at(pcl_index) << std::endl;
	//ROS_INFO("--> GET PCL");
	}
} catch (...) {
		ROS_WARN("Exception getPclCallback...");
    	return ;
	}
	
}


void addMarker(visualization_msgs::MarkerArray& m_array,float x, float y, float z, std::string label,std::string frame_id,int id) {
		visualization_msgs::Marker marker,markerTxt;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = "/world_management/pose_recorded";
		marker.id=id;

		//marker.color.a = 0.5;//initValue = 0.15
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 0.75;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.orientation.w=1;
		//markerN.pose.position.z = marker.scale.z / 2.0;;
		marker.pose.position.z = z;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.scale.x=0.05;
		marker.scale.y=0.05;
		marker.scale.z=0.05;
		
		/*markerTxt=marker;
		markerTxt.id=id+1000;
		markerTxt.pose.position.y=marker.pose.position.y+0.1;
		markerTxt.text=std::to_string(id);
		markerTxt.ns = "/world_management/pose_recorder_txt";
		markerTxt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		*/
		m_array.markers.push_back(marker);
		//m_array.markers.push_back(markerTxt);


}

geometry_msgs::Point processBox(geometry_msgs::Pose2D pose, std::string frame_id, std::string target_frame_id, ros::Time time_stamp,int32_t pixel_radius){
	geometry_msgs::Point resultPoint;

	int current_pixel_radius=pixel_radius;
	
	try{
		lock=true;
		int pcl_index,x,y,z;
		pcl::PointXYZ point;
		int rgb_w=pose.x;
		int rgb_h=pose.y;
		
		//std::string frame_id=frame_id;
		//std::string target_frame_id =target_frame_id;
		//pcl::PointXYZ target = pCloud.points[CoM.x + CoM.y * pCloud.width];


		pcl_index = ( (rgb_w*current_cloud->width) + rgb_h);

		//pcl_index = ( (rgb_w*current_cloud->height) + rgb_h);
		//check time elasped between PCL and current request

		
		pcl::fromROSMsg(*current_cloud,point_pcl);
		double pcl_time_sec =current_cloud->header.stamp.toSec();
		double current_req_sec=time_stamp.toSec();
		double diff=abs(pcl_time_sec-current_req_sec);
		if (diff < time_diff_threshold && current_req_sec!=0.0){
			ROS_WARN("No PCL Frame in sec diff time, expected<[%f] and got=[%f] : %s",time_diff_threshold,diff );
		}



        //FIXME get average of depht point into givent measure_radius
		float sum_x=0;
		float sum_y=0;
		float sum_z=0;
		int nb_pixel=0;
		//int base_index=( ((rgb_w)*current_cloud->width) + rgb_h);
		// point =point_pcl.at(pcl_index);
		//visualization_msgs::MarkerArray m_array;
		for(int i=-current_pixel_radius;i<=current_pixel_radius;i++){
			for(int j=-current_pixel_radius;j<=current_pixel_radius;j++){
				//ROS_INFO("--> BEFORE A found point base_index:[%i], current_index:[%i], rgb_w+i:[%i], rgb_h+j:[%i],h:[%i],w:[%i] ",base_index,pcl_index,rgb_w+i,rgb_h+j,current_cloud->height,current_cloud->width);
				if( (rgb_w+i>=0) &&  (rgb_h+j>=0) && (rgb_w+i<current_cloud->height) && (rgb_h+j<current_cloud->width)){
					pcl_index = ( ((rgb_w+i)*current_cloud->width) + rgb_h+j);
					//pcl_index = ( ((rgb_h+i)*current_cloud->height) + rgb_w+j);
					//ROS_INFO("--> inside measure_radius loop base_index:[%i], current_index:[%i], nb_pixel:[%i], radius:[%i]",base_index,pcl_index,nb_pixel,current_pixel_radius);
				    point =point_pcl.at(pcl_index);
					//ROS_INFO("--> BEFORE B found point base_index:[%i], current_index:[%i], x:[%f], y:[%f], z:[%f]",base_index,pcl_index,point.x,point.y,point.z);
		            if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z) && !isinf(point.x) && !isinf(point.y) && !isinf(point.z)){
				    //ROS_INFO("--> found point base_index:[%i], current_index:[%i], x:[%f], y:[%f], z:[%f]",base_index,pcl_index,point.x,point.y,point.z);
					//int id=std::rand();
					//addMarker(m_array,point.x,point.y,point.z,"test",frame_id,id);
					sum_x=sum_x+point.x;
					sum_y=sum_y+point.y;
					sum_z=sum_z+point.z;

					nb_pixel++;
			       }
				}
			}
		}
        if(nb_pixel==0){
			ROS_WARN("No pt to process around value x:%i,y:%i,radius:%i",rgb_w,rgb_h,current_pixel_radius);
			return resultPoint;
		}
		ROS_DEBUG("Sumx[%f],Sumy[%f],Sumz[%f]",sum_x,sum_y,sum_z);
		float avg_x = sum_x/float(nb_pixel);
		float avg_y = sum_y/float(nb_pixel);
		float avg_z = sum_z/float(nb_pixel);
		ROS_DEBUG("avg_x[%f],avg_y[%f],avg_z[%f]",avg_x,avg_y,avg_z);
		ROS_DEBUG("width[%i],height[%i]",current_cloud->width,current_cloud->height);

		//res.point.x=avg_x;
		//res.point.y=avg_y;
		//res.point.z=avg_z;

		//Calculating object position in the map frame
		geometry_msgs::PointStamped pt_stamp_in;
		pt_stamp_in.header.frame_id= frame_id;
		//pt_stamp_in.header.stamp=ros::Time::now();
		pt_stamp_in.point.x=avg_x;
		pt_stamp_in.point.y=avg_y;
		pt_stamp_in.point.z=avg_z;



		geometry_msgs::PointStamped pt_computed;
		tf::TransformListener listener;
		
        try{
			listener.waitForTransform(target_frame_id,frame_id, ros::Time(0), ros::Duration(5));
			listener.transformPoint(target_frame_id, pt_stamp_in, pt_computed);
        	//ROS_INFO("-->Pt a point optical ref : x:%f,y:%f",pt_stamp_in.point.x,pt_stamp_in.point.y);
        	//ROS_INFO("-->Pt a point map ref : x:%f,y:%f",pt_computed.point.x,pt_computed.point.y);

			resultPoint=pt_computed.point;
        	//pt_computed.header.frame_id = target_frame;

    	}
    	catch(tf::TransformException& ex){
    		ROS_WARN("[2D_TO_3D]Received an exception trying to transform a point : %s", ex.what());
      	}

		if(display_marker){
			visualization_msgs::MarkerArray m_array;
			int id=std::rand();
			addMarker(m_array,pt_computed.point.x,pt_computed.point.y,pt_computed.point.z,"test",target_frame_id,id);
			object_marker_pub.publish(m_array);
			//ROS_INFO("Maker Published...");
		}

		lock=false;
		// ...
	} catch (const std::exception &exc) {
		lock=false;
		ROS_WARN("[2D_TO_3D] Received an exception: %s",exc.what());
    	return resultPoint;
	}


	return resultPoint;
}

bool convert2dto3dCallback(convert_2d_to_3d::get3Dfrom2D::Request  &req,
             convert_2d_to_3d::get3Dfrom2D::Response &res){
				 if(current_cloud==0){
					 return false;
				 }
				//ROS_WARN("[2D_TO_3D]Ros param frame_id: %s, target_frame_id:%s", req.frame_id.c_str(),req.target_frame_id.c_str());
				 geometry_msgs::Point resultPoint=processBox(req.pose, req.frame_id, req.target_frame_id, req.stamp,req.pixel_radius);
				res.point = resultPoint;
	
	return true;
}

void getDarkNetBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
	robocup_msgs::EntityList currentEntityList;
	
	//ROS_INFO("--> in   getDarkNetBoxesCallback");
	try{
		for(int i=0;i<msg->bounding_boxes.size();i++){
			darknet_ros_msgs::BoundingBox bounding_box=msg->bounding_boxes[i];
			
			//FIXME manage filter on class
			//if(!filter.compare(bounding_box.Class)){
				robocup_msgs::Entity currentEntity;
				//ROS_INFO("--> in LOOP  getDarkNetBoxesCallback");
				
				currentEntity.label=bounding_box.Class;
				//ROS_INFO("--> after Class  getDarkNetBoxesCallback");
				geometry_msgs::Pose currentPose;

				//CAUTION check the round by the cast
  				geometry_msgs::Pose2D current_pose2d;
				int rgb_w=bounding_box.ymax-(bounding_box.ymax-bounding_box.ymin)/2;
				int rgb_h=bounding_box.xmax-(bounding_box.xmax-bounding_box.xmin)/2;
				current_pose2d.x=rgb_w;
 				current_pose2d.y=rgb_h;
				ros::Time current_stamp=ros::Time::now();



				//ROS_INFO("--> after ymin  getDarkNetBoxesCallback");
				//process 2d ->3d
				std::string frame_id=msg->header.frame_id;
				//FIXME to update with ros parameters
				currentEntity.header.frame_id="map";




				//OKcurrent_entity=Entity()
                //OKcurrent_entity.label=box.Class

              
                //OKcurrent_pose2d=Pose2D()
                //OKcurrent_pose2d.x=center[0]
                //OKcurrent_pose2d.y=center[1]
                //OK#rospy.loginfo(box)
                //OK#rospy.loginfo(current_pose2d)
                //OKcurrent_stamp =rospy.get_rostime()

				


				geometry_msgs::Point resultPoint=processBox(current_pose2d, frame_id, "map", current_stamp,10);


				currentEntity.type = "Object";
                currentEntity.pose.position.x=resultPoint.x;
                currentEntity.pose.position.y = resultPoint.y;
                currentEntity.pose.position.z = resultPoint.z;
				std::string json = 	"{\"confidence\" : "+ std::to_string(bounding_box.probability) +"}";
               // std::string json = '{"confidence":'+bounding_box.probability+'}';
                currentEntity.payload=json;
                currentEntityList.entityList.push_back(currentEntity);

				//int pcl_index;
				//pcl::PointXYZ point;
				//ROS_INFO("--> before ( (rgb_w*(current_cloud->width)) + rgb_h)  getDarkNetBoxesCallback");
				//if (current_cloud != NULL){
				//	pcl_index = ( (rgb_w*(current_cloud->width)) + rgb_h);
				//	ROS_INFO("--> before fromROSMsg  getDarkNetBoxesCallback");
				//	pcl::fromROSMsg(*current_cloud,point_pcl);
				//	ROS_INFO("--> after fromROSMsg  getDarkNetBoxesCallback");
				//	point =point_pcl.at(pcl_index);
				//	if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
				//			ROS_WARN("ISNAN value x:%f,y:%f,z:%f",point.x,point.y,point.z);
				//	}
//
//
				//	if(isinf(point.x) || isinf(point.y) || isinf(point.z)){
				//			ROS_WARN("ISINF value x:%f,y:%f,z:%f",point.x,point.y,point.z);
				//	}
//
				//	//TODO CONVERT INTO /MAP or /BASEFOOTPRING Frameid
//
				//	geometry_msgs::PointStamped pt_computed;
				//	pt_computed.header.frame_id=source_frame;
				//	pt_computed.header.stamp=ros::Time();
				//	pt_computed.point.x=point.x;
				//	pt_computed.point.y=point.y;
				//	pt_computed.point.z=point.z;
//
				//
        		//	geometry_msgs::PointStamped map_point;
				//
        		//	//need to transform coord from OPtical Frame to Base_Link or Map 
        		//	tf::TransformListener listener;
        		//	listener.waitForTransform(msg->header.frame_id, target_frame, ros::Time(0), ros::Duration(5));
        		//	try{
				//		listener.transformPoint(target_frame, pt_computed, map_point);
        		//		ROS_INFO("-->Pt a point optical ref : x:%f,y:%f",pt_computed.point.x,pt_computed.point.y);
        		//		ROS_INFO("-->Pt a point map ref : x:%f,y:%f",map_point.point.x,map_point.point.y);
        		//		pt_computed.point.x=map_point.point.x;
        		//		pt_computed.point.y=map_point.point.y;
        		//		pt_computed.header.frame_id = target_frame;
				//		currentPose.position.x=pt_computed.point.x;
				//		currentPose.position.y=pt_computed.point.y;
				//		currentPose.position.z=pt_computed.point.z;
				//		currentEntity.header.frame_id=target_frame;
//
    			//	}
    			//	catch(tf::TransformException& ex){
    			//		ROS_WARN("Received an exception trying to transform a point : %s", ex.what());
      			//	}
				//}
				

				//currentEntity.pose=currentPose;
//
				//currentEntityList.entityList.push_back(currentEntity);
				////ROS_INFO("--> before display_marker  getDarkNetBoxesCallback");
				//if(display_marker){
				//	visualization_msgs::MarkerArray m_array;
				//	int id=std::rand();
				//	addMarker(m_array,currentEntity.pose.position.x,currentEntity.pose.position.y,currentEntity.pose.position.z,"test",currentEntity.header.frame_id,id);
				//	object_marker_pub.publish(m_array);
				//}
			//}
		}

		// PUBLISH DATA
		object_entity_pub.publish(currentEntityList);
		ros::spinOnce();

	} catch (...) {
		ROS_WARN("Exception getDarkNetBoxesCallback...");
    	return ;
	}
}


/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"convert_2d_to_3d",ros::init_options::AnonymousName);
	ros::NodeHandle nh;
    
    ROS_INFO("Param values:");

	if (!ros::param::get("/convert_2d_to_3d/display_marker", display_marker))
		{
			display_marker=false;
		}
	ROS_INFO("- /convert_2d_to_3d/display_marker: %i",display_marker);

	if (!ros::param::get("/convert_2d_to_3d/filter_class", filter))
	    {
	      filter="person";
	    }

	ROS_INFO("- /convert_2d_to_3d/filter_class: %s", filter.c_str());

	if (!ros::param::get("/convert_2d_to_3d/target_frame", target_frame))
	    {
	      target_frame="base_footprint";
	    }

	ROS_INFO("- /convert_2d_to_3d/target_frame: %s", target_frame.c_str());

	if (!ros::param::get("/convert_2d_to_3d/source_frame", source_frame))
	    {
	      source_frame="CameraTop_optical_frame";
	    }
	ros::param::get("~source_frame", source_frame);
	ROS_INFO("- /convert_2d_to_3d/source_frame: %s", source_frame.c_str());

	if (!ros::param::get("/convert_2d_to_3d/pcl_topic", pcl_topic))
	    {
	      pcl_topic="/kinect/depth/points";
	    }
	ros::param::get("~pcl_topic", pcl_topic);
    ROS_INFO("- /convert_2d_to_3d/pcl_topic: %s", pcl_topic.c_str());

	if (!ros::param::get("/convert_2d_to_3d/measure_radius", measure_radius))
	    {
	      measure_radius=10;
	    }
	
	ROS_INFO("- /convert_2d_to_3d/measure_radius: %i", measure_radius);

		if (!ros::param::get("/convert_2d_to_3d/time_diff_threshold", time_diff_threshold))
	    {
	      time_diff_threshold=0.05;
	    }
	
	ROS_INFO("- /convert_2d_to_3d/time_diff_threshold: %f", time_diff_threshold);

    if (!ros::param::get("/convert_2d_to_3d/service_name", service_name))
	    {
	      service_name="convert_2d_to_3d";
	    }
	ros::param::get("~/service_name", service_name);
	ROS_INFO("- /convert_2d_to_3d/service_name: %f", service_name);

	

	

	// Publisher
	object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/objet_pose_registred_pub", 1);
	object_entity_pub = nh.advertise<robocup_msgs::EntityList>("/world_mng/objects/entity", 1);
	sub_registered_pcl= nh.subscribe(pcl_topic, 1, getPclCallback);
	sub_registered_darknetBoxes=nh.subscribe("/darknet_ros/bounding_boxes",5, getDarkNetBoxesCallback);
	ros::ServiceServer service = nh.advertiseService(service_name, convert2dto3dCallback);
   	ROS_INFO("Ready to convert rgb coord to pcl 3d point.");
    ros::spin();

}
