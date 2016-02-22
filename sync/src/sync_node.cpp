#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"

#include <sstream>
#include <string>
#include <iostream>

std_msgs::Float32MultiArray array;
sensor_msgs::Imu wearable_1;
sensor_msgs::Imu wearable_2;
bool flagMotion = false;
bool flagKinect = false;
bool flagWearable_1 = false;
bool flagWearable_2 = false;

using namespace std;


void coordinates_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	array.data = msg->data;
	flagMotion = true;
}

void wearable_callback_1(const sensor_msgs::Imu::ConstPtr& msg)
{
	wearable_1.angular_velocity.x = msg->angular_velocity.x;
	wearable_1.angular_velocity.y = msg->angular_velocity.y;
	wearable_1.angular_velocity.z = msg->angular_velocity.z;
	
	wearable_1.linear_acceleration.x = msg->linear_acceleration.x;
	wearable_1.linear_acceleration.y = msg->linear_acceleration.y;
	wearable_1.linear_acceleration.z = msg->linear_acceleration.z;
	flagWearable_1 = true;
}

void wearable_callback_2(const sensor_msgs::Imu::ConstPtr& msg)
{
	wearable_2.angular_velocity.x = msg->angular_velocity.x;
	wearable_2.angular_velocity.y = msg->angular_velocity.y;
	wearable_2.angular_velocity.z = msg->angular_velocity.z;
	
	wearable_2.linear_acceleration.x = msg->linear_acceleration.x;
	wearable_2.linear_acceleration.y = msg->linear_acceleration.y;
	wearable_2.linear_acceleration.z = msg->linear_acceleration.z;
	flagWearable_2 = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sync_node");
	ros::NodeHandle nh;
	
	ros::Subscriber coordinates = nh.subscribe("/markers_coo",1, coordinates_callback);
	ros::Subscriber wearableSensor_1 = nh.subscribe("/wearS_1",1, wearable_callback_1);
	ros::Subscriber wearableSensor_2 = nh.subscribe("/wearS_2",1, wearable_callback_2);

	
	tf::TransformListener listener;
		
	rosbag::Bag bag;
	
	string file_name;
	cout << "Nome file: "<<endl;
	cin >> file_name;
	
	bag.open(file_name, rosbag::bagmode::Write);
	
	while(ros::ok()){
		
		tf::StampedTransform transform[15];
		
		for(int i=0; i < 15; i++){
			transform[i].setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		}
		
		try{
        	listener.lookupTransform("/openni_depth_frame", "right_foot_1",  
            ros::Time(0), transform[0]);
			listener.lookupTransform("/openni_depth_frame", "right_knee_1",  
            ros::Time(0), transform[1]);
			listener.lookupTransform("/openni_depth_frame", "right_hip_1",  
            ros::Time(0), transform[2]);
			listener.lookupTransform("/openni_depth_frame", "right_hand_1",  
            ros::Time(0), transform[3]);
			listener.lookupTransform("/openni_depth_frame", "right_elbow_1",  
            ros::Time(0), transform[4]);
			listener.lookupTransform("/openni_depth_frame", "right_shoulder_1",  
            ros::Time(0), transform[5]);
			listener.lookupTransform("/openni_depth_frame", "left_foot_1",  
            ros::Time(0), transform[6]);
			listener.lookupTransform("/openni_depth_frame", "left_knee_1",  
            ros::Time(0), transform[7]);
			listener.lookupTransform("/openni_depth_frame", "left_hip_1",  
            ros::Time(0), transform[8]);
			listener.lookupTransform("/openni_depth_frame", "left_hand_1",  
            ros::Time(0), transform[9]);
			listener.lookupTransform("/openni_depth_frame", "left_elbow_1",  
            ros::Time(0), transform[10]);
			listener.lookupTransform("/openni_depth_frame", "left_shoulder_1",  
            ros::Time(0), transform[11]);
			listener.lookupTransform("/openni_depth_frame", "torso_1",  
            ros::Time(0), transform[12]);
			listener.lookupTransform("/openni_depth_frame", "neck_1",  
            ros::Time(0), transform[13]);
			listener.lookupTransform("/openni_depth_frame", "head_1",  
            ros::Time(0), transform[14]);
			flagKinect = true;
      	}
      	catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
			flagKinect = false;
        }
		
				
		if(flagMotion & flagKinect & flagWearable_2 & flagWearable_1 ){
			ROS_INFO("LINE");
			
			int count = 1;
			ros::Time current = ros::Time::now();
			geometry_msgs::Point co;
			
			for(std::vector<float>::const_iterator it = array.data.begin(); it != array.data.end(); ++it){

				co.x = *it;
				++it;
				co.y = *it;
				++it;
				co.z = *it;
				
				ostringstream Xos;
				Xos << "P_" << count;
				string Xs = Xos.str();
				bag.write(Xs, current, co);

				++count;
			}
			
			
			//RIGHT FOOT
			co.x = transform[0].getOrigin().x();
			co.y = transform[0].getOrigin().y();
			co.z = transform[0].getOrigin().z();
			bag.write("right_foot", current, co);
				
			//RIGHT KNEE
			co.x = transform[1].getOrigin().x();
			co.y = transform[1].getOrigin().y();
			co.z = transform[1].getOrigin().z();
			bag.write("right_knee", current, co);
				
			//RIGHT HIP
			co.x = transform[2].getOrigin().x();
			co.y = transform[2].getOrigin().y();
			co.z = transform[2].getOrigin().z();
			bag.write("right_hip", current, co);
				
			//RIGHT HAND
			co.x = transform[3].getOrigin().x();
			co.y = transform[3].getOrigin().y();
			co.z = transform[3].getOrigin().z();
			bag.write("right_hand", current, co);
				
			//RIGHT ELBOW
			co.x = transform[4].getOrigin().x();
			co.y = transform[4].getOrigin().y();
			co.z = transform[4].getOrigin().z();
			bag.write("right_elbow", current, co);
				
			//RIGHT SHOULDER
			co.x = transform[5].getOrigin().x();
			co.y = transform[5].getOrigin().y();
			co.z = transform[5].getOrigin().z();
			bag.write("right_shoulder", current, co);
				
			//LEFT FOOT
			co.x = transform[6].getOrigin().x();
			co.y = transform[6].getOrigin().y();
			co.z = transform[6].getOrigin().z();
			bag.write("left_foot", current, co);
				
			//LEFT KNEE
			co.x = transform[7].getOrigin().x();
			co.y = transform[7].getOrigin().y();
			co.z = transform[7].getOrigin().z();
			bag.write("left_knee", current, co);
				
			//LEFT HIP
			co.x = transform[8].getOrigin().x();
			co.y = transform[8].getOrigin().y();
			co.z = transform[8].getOrigin().z();
			bag.write("left_hip", current, co);
				
			//LEFT HAND
			co.x = transform[9].getOrigin().x();
			co.y = transform[9].getOrigin().y();
			co.z = transform[9].getOrigin().z();
			bag.write("left_hand", current, co);
				
			//LEFT ELBOW
			co.x = transform[10].getOrigin().x();
			co.y = transform[10].getOrigin().y();
			co.z = transform[10].getOrigin().z();
			bag.write("left_elbow", current, co);
				
			//LEFT SHOULDER
			co.x = transform[11].getOrigin().x();
			co.y = transform[11].getOrigin().y();
			co.z = transform[11].getOrigin().z();
			bag.write("left_shoulder", current, co);
				
			//TORSO
			co.x = transform[12].getOrigin().x();
			co.y = transform[12].getOrigin().y();
			co.z = transform[12].getOrigin().z();
			bag.write("torso", current, co);
				
			//NECK
			co.x = transform[13].getOrigin().x();
			co.y = transform[13].getOrigin().y();
			co.z = transform[13].getOrigin().z();
			bag.write("neck", current, co);
				
			//HEAD
			co.x = transform[14].getOrigin().x();
			co.y = transform[14].getOrigin().y();
			co.z = transform[14].getOrigin().z();
			bag.write("head", current, co);
			
			//Wearable Sensors data
			bag.write("wearable_1", current, wearable_1);
			bag.write("wearable_2", current, wearable_2);
			
			
			flagMotion = false;
			flagKinect = false;
			flagWearable_1 = false;
			flagWearable_2 = false;
		}
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	
	bag.close();
	
	return 0;
}

