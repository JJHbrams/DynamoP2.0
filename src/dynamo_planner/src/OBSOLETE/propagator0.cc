#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
	class propagator : public ModelPlugin
	{
		private: physics::ModelPtr model;
		private: physics::JointController * jcX;
		private: physics::JointController * jcY;
		private: physics::JointController * jcYaw;
		private: physics::JointController * jcFLW;
		private: physics::JointController * jcFRW;
		private: physics::JointController * jcRLW;
		private: physics::JointController * jcRRW;
		private: physics::JointPtr jX;	//base X
		private: physics::JointPtr jY;	//base Y
		private: physics::JointPtr jYaw;// base Yaw
		private: physics::JointPtr jFLW;//Front Left Wheel
		private: physics::JointPtr jFRW;//Front Right Wheel
		private: physics::JointPtr jRLW;//Rear Left Wheel
		private: physics::JointPtr jRRW;//Rear Right Wheel

		private: common::PID pidX;
		private: common::PID pidY;
		private: common::PID pidYaw;
		private: common::PID pidFLW;
		private: common::PID pidFRW;
		private: common::PID pidRLW;
		private: common::PID pidRRW;

		private: event::ConnectionPtr updateConnection;
		private: ros::NodeHandle nh;
		private: ros::Subscriber sub;

		private: float data_buf[7];
		private: int time;

		public: propagator(){}
		public: ~propagator(){
			ROS_INFO("Done");
			this->nh.shutdown();
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
		{

			ROS_INFO("Load propagator!");
			std::cerr << "\nThe propagator plugin is attach to model[" <<	_parent->GetName() << "]\n";

			// Safety check
		  if (_parent->GetJointCount() == 0)
		  {
		    std::cerr << "Invalid joint count, plugin not loaded\n";
		    return;
		  }

			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "propagator");
			// sub = nh.subscribe("/joint_states", 1, &husky::cb, this);
			// sub = nh.subscribe("/tf", 1, &husky::cb, this);
			// sub = nh.subscribe("dynamop/path_pose", 1, &husky::cb, this);

			model = _parent;
			jcX = new physics::JointController(model);
			jcY = new physics::JointController(model);
			jcYaw = new physics::JointController(model);
			jcFLW = new physics::JointController(model);
			jcFRW = new physics::JointController(model);
			jcRLW = new physics::JointController(model);
			jcRRW = new physics::JointController(model);

			// jX = model->GetJoint("base_x_joint");
			// jY = model->GetJoint("base_y_joint");
			// jYaw = model->GetJoint("base_yaw_joint");

			jX = model->GetJoints()[1];
			jY = model->GetJoints()[2];
			jYaw = model->GetJoints()[3];

			jFLW = model->GetJoints()[4];
			jFRW = model->GetJoints()[5];
			jRLW = model->GetJoints()[6];
			jRRW = model->GetJoints()[7];

			pidX = common::PID(0.1,0,0);
			pidY = common::PID(0.1,0,0);
			pidYaw = common::PID(0.1,0,0);

			pidFLW = common::PID(0.1,0,0);
			pidFRW = common::PID(0.1,0,0);
			pidRLW = common::PID(0.1,0,0);
			pidRRW = common::PID(0.1,0,0);

			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&propagator::OnUpdate,
										this, _1));
			time = 0;
			data_buf[0] = 0;
			data_buf[1] = 0;
			data_buf[2] = 0;
			data_buf[3] = 0;
			data_buf[4] = 0;
			data_buf[5] = 0;
			data_buf[6] = 0;
		}

		public: void cb(const sensor_msgs::JointState::ConstPtr &msg)
		// public: void cb(const tf2_msgs::TFMessage::ConstPtr &msg)
		// public: void cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
		{
			// ROS_INFO("run cb()");
			// Subscribe /joint_states
			data_buf[0] = msg->position[0];
			data_buf[1] = msg->position[1];
			data_buf[2] = msg->position[2];
			data_buf[3] = msg->position[3];
			data_buf[4] = msg->position[4];
			data_buf[5] = msg->position[5];
			data_buf[6] = msg->position[6];

			// Subscribe /tf
			// data_buf[0] = msg->transforms.at(0).transform.translation.x;
			// data_buf[1] = msg->transforms.at(1).transform.translation.y;
			// data_buf[2] = msg->transforms.at(2).transform.rotation.w;
			// if(data_buf[2] > 0)	data_buf[2] = -data_buf[2];
			// data_buf[2] = (data_buf[2]+1)*M_PI;
			// for(int i = 0; i < msg->transforms.size(); i++){
			// 	ROS_INFO("%d, %f, %f, %f, %f, %f, %f, %f"	,msg->transforms.size()
			// 																				,msg->transforms.at(i).transform.translation.x
			// 																				,msg->transforms.at(i).transform.translation.y
			// 																				,msg->transforms.at(i).transform.translation.z
			// 																				,msg->transforms.at(i).transform.rotation.w
			// 																				,msg->transforms.at(i).transform.rotation.x
			// 																				,msg->transforms.at(i).transform.rotation.y
			// 																				,msg->transforms.at(i).transform.rotation.z);
	    // }

			// ROS_INFO("x : %f, y : %f, yaw : %f", data_buf[0], data_buf[1], data_buf[2]);
		}

		public: void OnUpdate(const common::UpdateInfo &)
		{
			jcX->SetJointPosition(jX, data_buf[0]);
			jcY->SetJointPosition(jY, data_buf[1]);
			jcYaw->SetJointPosition(jYaw, data_buf[2]);
			jcFLW->SetJointPosition(jFLW, data_buf[3]);
			jcFRW->SetJointPosition(jFRW, data_buf[4]);
			jcRLW->SetJointPosition(jRLW, data_buf[5]);
			jcRRW->SetJointPosition(jRRW, data_buf[6]);
		}
	};
	GZ_REGISTER_MODEL_PLUGIN(propagator)
}
