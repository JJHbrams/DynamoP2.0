#include <ros/ros.h>

#include <ode/ode.h>

#include <iostream>
#include <fstream>
#include <random>

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
		private: ros::Publisher gazeboCtrl, SamplerState;
		private: transport::PublisherPtr gazeboStep, gazeboPhysics;

		private: geometry_msgs::Twist vel;
		private: dynamo_planner::data_gen_msgs State;

		private: msgs::WorldControl StepMsg;
		private: msgs::Physics physicsMsg;

		private: int NUM_SAMPLES=0;
		private: double X_set, Y_set, YAW_set, speedA, speedB, TIME_set;
		private: double SIM_TIME;
		private: std::ofstream dataout;

		public: propagator(){}
		public: ~propagator(){
			ROS_INFO("Done");
			this->nh.shutdown();
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)//Initialization
		{
			ROS_INFO("Load physics sampler!");

			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "physics_data_sampler");
			transport::NodePtr node(new gazebo::transport::Node());
			node->Init();
			//Subscriber
			planner2gazebo 	= nh.subscribe("/Run", 1, &propagator::P2G_cb, this);
			sub_js 					= nh.subscribe("/joint_states", 1, &propagator::js_cb, this);
			//Publisher
			gazeboCtrl 			= nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1, true);
			SamplerState	= nh.advertise<dynamo_planner::data_gen_msgs>("/Done", 1, true);
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
      physicsMsg.set_max_step_size(0.001);
			gazeboPhysics->Publish(physicsMsg);

			// std::cerr << "\nThe propagator plugin is attach to model[" <<	model->GetName() << "], world[" << world->GetName() <<"]\n";
			ROS_INFO("World name : %s", world->Name().c_str());
			ROS_INFO("Robot model : %s", model->GetName().c_str());
			ROS_INFO("Physics engine : %s", physics_engine->GetType().c_str());
			ROS_INFO("Time step size : %f", physics_engine->GetMaxStepSize());//In case of ODE, it is time step size
			ROS_INFO("Update period : %f", physics_engine->GetUpdatePeriod());
			SIM_TIME = physics_engine->GetMaxStepSize();

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

			updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&propagator::OnUpdate, this, _1));
		}

		int sgn(double Val){
			if(Val==0)  return 0;
			else        return ((Val > 0) - (Val < 0));
		}

		std::mt19937 generator;
		double RAND_VAL(double MIN, double MAX, char MODE){
			// Normaldistribution
			double mean = (MAX+MIN)/2;
			double stddev  = 1;
			switch(MODE){
				case 'u':{
					std::uniform_real_distribution<double> uniform(mean, stddev);
					srand(uniform(generator)*(time(NULL)));
					break;
				}

				case 'n':{
					std::normal_distribution<double> normal(mean, stddev);
					srand(normal(generator)*(time(NULL)));
					break;
				}

			}

			// std::cerr << "Normal: " << normal(generator) << std::endl;
			double RES = 1E4;
			int DIV = static_cast<int>((MAX-MIN)*RES+1);
			double SHIFT = ((MAX-MIN)/2.)*RES;
			double MEAN = (MAX+MIN)/2*RES;

			return (rand()%DIV - SHIFT + MEAN)/RES;
		}

		void js_cb(const sensor_msgs::JointState::ConstPtr& _msg){
			// std::cout << "tf_cb" << std::endl;
			if(State.flag==true){
				// For yaw larger than M_PI
				int sign = sgn(_msg->position[2]);
				double val_rad = abs(_msg->position[2]);
				while(val_rad > M_PI)	val_rad -= M_PI;

				if(dataout.is_open()){
				  dataout << "         " << ", " << "Xa"    << ","  << "Ya"    << ", " << "YAWa"<< ", " << "CTRLA"<< ", " << "CTRLB"<< ", " << "Duration"    << ", " <<"Xb"      << ", " << "Yb"     << ", " << "YAWb"<<"\n";
				  dataout << NUM_SAMPLES << ", " << X_set << ", " << Y_set << ", " << YAW_set  << ", " << speedA << ", " << speedB << ", " << TIME_set*SIM_TIME << ", " << _msg->position[0] << ", " << _msg->position[1] << ", " << sign*val_rad << "\n";
				}
				NUM_SAMPLES++;
				State.flag=false;
			}
		}

		public: void P2G_cb(const dynamo_planner::data_gen_msgs& _msg)
		{
			dataout.open(_msg.filename);

			if(_msg.flag == true){

				for(int Loop=0; Loop<_msg.num_data; Loop++){

					X_set = RAND_VAL(-5,5,'u'), Y_set = RAND_VAL(-5,5,'u'), YAW_set = RAND_VAL(-M_PI,M_PI,'u');
					speedA = RAND_VAL(-100.0,100.0,'n'), speedB = RAND_VAL(-100.0,100.0,'n');
					TIME_set = RAND_VAL(100, 5000,'u');

					X_set=0;Y_set=0;YAW_set=0;
					// speedA=100;speedB=100;
					// TIME_set=500;

					jcX->SetJointPosition(jX, X_set);
					jcY->SetJointPosition(jY, Y_set);
					jcZ->SetJointPosition(jZ, 0.17);//Wheel radius
					jcYaw->SetJointPosition(jYaw, YAW_set);

					vel.linear.x = (speedA+speedB)*cos(YAW_set);
					vel.linear.y = (speedA+speedB)*sin(YAW_set);
					vel.angular.z = speedB-speedA;
					gazeboCtrl.publish(vel);

					int NUM_STEP = TIME_set;
					StepMsg.set_multi_step(NUM_STEP);
					gazeboStep->Publish(StepMsg);

					gazebo::common::Time::MSleep(TIME_set);

					State.flag=true;

					vel.linear.x = 0.0;
					vel.linear.y = 0.0;
					vel.angular.z = 0.0;
					gazeboCtrl.publish(vel);
					// ros::Time begin = ros::Time::now();
					// ros::Time Now = ros::Time::now();
					// while((Now.toSec() - begin.toSec()) < 0.1);

					// StepMsg.set_step(true);
					// gazeboStep->Publish(StepMsg);

					// NUM_SAMPLES++;
				}
			}

			dataout.close();
			State.flag=true;
			SamplerState.publish(State);
			State.flag=false;
		}


		public: void OnUpdate(const common::UpdateInfo &)
		{

		}
	};
	GZ_REGISTER_MODEL_PLUGIN(propagator)
}
