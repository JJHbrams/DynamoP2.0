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
#include <dynamo_planner/custom_states_msgs.h>
#include <geometry_msgs/Twist.h>

#include <boost/bind.hpp>

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
		private: transport::PublisherPtr gazeboStep, gazeboPhysics;

		private: double posX, posY, posYAW;
		private: double ctrlX, ctrlY, ctrlYAW;
		private: double time;
		private: geometry_msgs::Twist vel;

		private: dynamo_planner::custom_states_msgs PropStates;

		private: msgs::WorldControl StepMsg;
		private: msgs::Physics physicsMsg;

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
			transport::NodePtr node(new gazebo::transport::Node());
			node->Init();
			//Subscriber
			planner2gazebo 	= nh.subscribe("/PlannerStates", 1, &propagator::P2G_cb, this);
			sub_js 					= nh.subscribe("/joint_states", 1, &propagator::js_cb, this);
			// sub_tf 					= nh.subscribe("/tf", 1, &propagator::tf_cb, this);
			//Publisher
			gazeboCtrl 			= nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1, true);
			gazebo2planner	= nh.advertise<dynamo_planner::custom_states_msgs>("/Prop_States", 1, true);
			gazeboStep 			= node->Advertise<msgs::WorldControl>("~/world_control");
			gazeboPhysics 	= node->Advertise<msgs::Physics>("~/physics");
			gazeboStep->WaitForConnection();
			gazeboPhysics->WaitForConnection();

			model = _parent;
			world = model->GetWorld();
			physics_engine = world->Physics();
			//Set physic engine
			physicsMsg.set_type(msgs::Physics::ODE);
			// Set the step time
      physicsMsg.set_max_step_size(0.01);
			gazeboPhysics->Publish(physicsMsg);

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

		void js_cb(const sensor_msgs::JointState::ConstPtr& _msg){
			// std::cout << "tf_cb" << std::endl;
			if(PropStates.flag==true){
				PropStates.x = _msg->position[0];
				PropStates.y = _msg->position[1];
				PropStates.yaw = _msg->position[2];
				gazebo2planner.publish(PropStates);
				PropStates.flag=false;
			}
		}

		public: void P2G_cb(const dynamo_planner::custom_states_msgs& _msg)
		{
			if(PropStates.flag != true){

				PropStates.pre_x = _msg.x;
				PropStates.pre_y = _msg.y;
				PropStates.pre_yaw = _msg.yaw;

				jcX->SetJointPosition(jX, _msg.x);
				jcY->SetJointPosition(jY, _msg.y);
				jcZ->SetJointPosition(jZ, 0.1651);//Wheel radius
				jcYaw->SetJointPosition(jYaw, _msg.yaw);

				vel.linear.x = (_msg.controlA+_msg.controlB)*cos(_msg.yaw);
				vel.linear.y = (_msg.controlA+_msg.controlB)*sin(_msg.yaw);
				vel.angular.z = _msg.controlB-_msg.controlA;
				gazeboCtrl.publish(vel);

				int NUM_STEP = _msg.duration/(physics_engine->GetMaxStepSize());
				StepMsg.set_multi_step(NUM_STEP);
				gazeboStep->Publish(StepMsg);

				double begin = ros::Time::now().toSec();
				double Now = ros::Time::now().toSec();
				while((Now - begin) < 0.1){
					Now = ros::Time::now().toSec();
					std::cout << "Begin : " << begin << std::endl;
					std::cout << "Now   : " << Now << std::endl;
				}
				// StepMsg.set_step(true);
				// gazeboStep->Publish(StepMsg);

				PropStates.flag=true;
			}
		}


		public: void OnUpdate(const common::UpdateInfo &)
		{

		}
	};
	GZ_REGISTER_MODEL_PLUGIN(propagator)
}
