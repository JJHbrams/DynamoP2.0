#include <ros/ros.h>

#include <ode/ode.h>

#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "dynamo_planner/data_gen_msgs.h"
#include <geometry_msgs/Twist.h>

#include <boost/bind.hpp>

#define NUM_TR 300000
#define NUM_TE 5000


dynamo_planner::data_gen_msgs order;

void sampler_callback(const dynamo_planner::data_gen_msgs& _msg){
	if(_msg.flag == true){
		order.flag=false;
		std::cout << "Done!" << std::endl;
	}
}

int main(int argc, char** argv){
	ROS_INFO("URDF_sampler!");
	ros::init(argc, argv, "URDF_sampler");

	ros::NodeHandle node_handle("~");
	ros::Publisher  pub_  = node_handle.advertise<dynamo_planner::data_gen_msgs>("/Run", 1);
	ros::Subscriber sub_  = node_handle.subscribe("/Done", 1, sampler_callback);

	order.flag = false;

	char Mode;
	do{
		if(order.flag == false){
			std::cout << "\nSelect the mode [1]Train data [2]Test data" << std::endl;
			std::cin >> Mode;
			if(Mode == '1'){
				// Training data generation
				std::cout << "Make train dataset..." << std::endl;
				order.filename = "/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/data/TR_two_wheeled_URDF.txt";
				order.num_data = NUM_TR;
			}
			else if(Mode == '2'){
				// Test data generation
				std::cout << "Make test dataset..." << std::endl;
				order.filename = "/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/data/TE_two_wheeled_URDF.txt";
				order.num_data = NUM_TE;
			}
			else{
				std::cout << "Wrong input..." << std::endl;
				order.flag = false;
				continue;
			}
		}
		order.flag = true;
		pub_.publish(order);
		ros::spinOnce();

	}while(ros::ok());

	return 0;
}
