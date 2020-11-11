#include <ode/ode.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "dynamo_planner/custom_states_msgs.h"
#include <geometry_msgs/Twist.h>
// #include <gazebo/physics/ode/ode_inc.h>

namespace gazebo
{
	class propagator : public ModelPlugin
	{
		private: physics::ModelPtr model;
		private: physics::WorldPtr world;
		private: physics::PhysicsEnginePtr physics_engine;
		private: physics::JointController * jcX;
		private: physics::JointController * jcY;
		private: physics::JointController * jcZ;
		private: physics::JointController * jcYaw;
		private: physics::JointPtr jX;	//base X
		private: physics::JointPtr jY;	//base Y
		private: physics::JointPtr jZ;	//base Z
		private: physics::JointPtr jYaw;// base Yaw

		private: event::ConnectionPtr updateConnection;
		private: ros::NodeHandle nh;
		private: ros::Subscriber planner2gazebo, sub_tf, sub_js;
		private: ros::Publisher gazeboCtrl, gazebo2planner;

		private: double posX, posY, posYAW;
		private: double ctrlX, ctrlY, ctrlYAW;
		private: double time;
		private: geometry_msgs::Twist vel;

		private: dynamo_planner::custom_states_msgs PropStates;

		public: propagator(){}
		public: ~propagator(){
			ROS_INFO("Done");
			this->nh.shutdown();
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)//Initialization
		{
			ROS_INFO("Load propagator!");

			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "propagator");
			//Subscriber
			planner2gazebo 	= nh.subscribe("/PlannerStates", 1, &propagator::P2G_cb, this);
			// sub_js 					= nh.subscribe("/joint_states", 1, &propagator::js_cb, this);
			// sub_tf 					= nh.subscribe("/tf", 1, &propagator::tf_cb, this);
			//Publisher
			gazeboCtrl 			= nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1, true);
			gazebo2planner	= nh.advertise<dynamo_planner::custom_states_msgs>("/GazeboStates", 1, true);

			model = _parent;
			world = model->GetWorld();
			physics_engine = world->Physics();

			// std::cerr << "\nThe propagator plugin is attach to model[" <<	model->GetName() << "], world[" << world->GetName() <<"]\n";
			ROS_INFO("World name : %s", world->Name().c_str());
			ROS_INFO("Robot model : %s", model->GetName().c_str());
			ROS_INFO("Physics engine : %s", physics_engine->GetType().c_str());
			ROS_INFO("Time step size : %f", physics_engine->GetMaxStepSize());//In case of ODE, it is time step size
			ROS_INFO("Update period : %f", physics_engine->GetUpdatePeriod());

			// Safety check
		  if (model->GetJointCount() == 0)
		  {
		    std::cerr << "Invalid joint count, plugin not loaded\n";
		    return;
		  }

			jcX = new physics::JointController(model);
			jcY = new physics::JointController(model);
			jcZ = new physics::JointController(model);
			jcYaw = new physics::JointController(model);

			jX = model->GetJoints()[1];
			jY = model->GetJoints()[2];
			jZ = model->GetJoints()[3];
			jYaw = model->GetJoints()[4];

			std::cout << "***Robot joint name***" << std::endl;
			std::cout << jX->GetName() << std::endl;
			std::cout << jY->GetName() << std::endl;
			std::cout << jZ->GetName() << std::endl;
			std::cout << jYaw->GetName() << std::endl;
			std::cout << "***********************" << std::endl;

			time = 0;
			//Position
			posX = 0;
			posY = 0;
			posYAW = 0;
			//Control
			ctrlX = 0;
			ctrlY = 0;
			ctrlYAW = 0;

			PropStates.x = 0.0;
	    PropStates.y = 0.0;
	    PropStates.yaw = 0.0;
			PropStates.flag = false;

			updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&propagator::OnUpdate, this, _1));

		}

		public: void P2G_cb(const dynamo_planner::custom_states_msgs& _msg)
		{
			// ROS_INFO("run cb()");
			// Get states from planner
			//*Positions
			/*posX = _msg.x;
	    posY = _msg.y;
	    posYAW = _msg.yaw;
			//*Controls
			ctrlX = _msg.controlX;
	    ctrlY = _msg.controlY;
	    ctrlYAW = _msg.controlYAW;
			//*Duration
			time = _msg.duration;*/
			// std::cout << "Given Position  x : " << _msg.x << ", y : " << _msg.y << ", yaw : " << _msg.yaw << std::endl;
			// std::cout << "Given Control   x : " << ctrlX << ", y : " << ctrlY << ", yaw : " << ctrlYAW << std::endl;
			// std::cout << "Given Duration 	 t : " << time << std::endl;

			//Spawn robot current position
			jcX->SetJointPosition(jX, _msg.x);
			jcY->SetJointPosition(jY, _msg.y);
			jcZ->SetJointPosition(jZ, 0.1651);//Wheel radius
			jcYaw->SetJointPosition(jYaw, _msg.yaw);

			PropStates.pre_x = _msg.x;
			PropStates.pre_y = _msg.y;
			PropStates.pre_yaw = _msg.yaw;
			//Propagate
			vel.linear.x = _msg.controlX;
			vel.linear.y = _msg.controlY;
			vel.angular.z = _msg.controlYAW;

			double begin = ros::Time::now().toSec();
			while((ros::Time::now().toSec() - begin) <= _msg.duration){
				// std::cout << ros::Time::now().toSec() - begin << ", " << _msg.duration << std::endl;
				// ROS_INFO("Step time : %f", physics_engine->GetStepTime());
				gazeboCtrl.publish(vel);//kinemati model
			}
			// std::cout << "Calculation is done!" << time << std::endl;

			/*PropStates.x = _msg.x + _msg.controlX*_msg.duration;
			PropStates.y = _msg.y + _msg.controlY*_msg.duration;
			PropStates.yaw = _msg.yaw + _msg.controlYAW*_msg.duration;*/
			// std::cout << "Propagated Position 	 x : " << PropStates.x << ", y : " << PropStates.y << ", yaw : " << PropStates.yaw << std::endl;

			PropStates.flag = true;
			// std::cout << "PropStates.flag : " << int(PropStates.flag) << std::endl;
			gazebo2planner.publish(PropStates);
			PropStates.flag = false;
		}
		/*public: void js_cb(const sensor_msgs::JointState::ConstPtr& _msg)
		{
			// std::cout << "tf_cb" << std::endl;
			PropStates.x = _msg->position[0];
			PropStates.y = _msg->position[1];
			PropStates.yaw = _msg->position[3];
			// PropStates.x = _msg->velocity[0];
			// PropStates.y = _msg->velocity[1];
			// PropStates.yaw = _msg->velocity[3];

			if(PropStates.flag)	gazebo2planner.publish(PropStates);
			PropStates.flag = false;
		}*/
		/*public: void tf_cb(const tf2_msgs::TFMessage::ConstPtr& _msg)
		{
			// std::cout << "tf_cb" << std::endl;
			PropStates.x = _msg->transforms.at(0).transform.translation.x;
			PropStates.y = _msg->transforms.at(1).transform.translation.y;
			PropStates.yaw = _msg->transforms.at(2).transform.rotation.w;

			PropStates.tf_flag = true;
		}*/

		public: void OnUpdate(const common::UpdateInfo &)
		{
			// ROS_INFO("OnUpdate()");
			//Update lately estimated position
			/*jcX->SetJointPosition(jX, 0.0);
			jcY->SetJointPosition(jY, 0.0);
			jcZ->SetJointPosition(jZ, 0.1651);
			jcYaw->SetJointPosition(jYaw, -3.14);*/


			// std::cout << "PropStates.flag : " << int(PropStates.flag) << std::endl;
			/*if(PropStates.flag && PropStates.tf_flag){
				gazebo2planner.publish(PropStates);
				PropStates.flag = false;
				PropStates.tf_flag = false;
			}*/
		}
	};
	GZ_REGISTER_MODEL_PLUGIN(propagator)
}
