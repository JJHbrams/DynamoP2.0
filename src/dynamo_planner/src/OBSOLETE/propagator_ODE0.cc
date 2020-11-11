// Propagator using Open Dynamic Engine
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
#include <math.h>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen_conversions/eigen_msg.h>
// #include <gazebo/physics/ode/ode_inc.h>

#include <drawstuff/drawstuff.h>
#include "../../ode/ode/demo/texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// ODE buggy parameters
#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.5	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass

namespace gazebo
{
	class propagator : public ModelPlugin
	{
		// GZEBO settings
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

		// ODE settings
		private: static dWorldID dworld;
		private: static dSpaceID dspace;
		private: static dBodyID body[4];
		private: static dJointID joint[16];	// joint[0] is the front wheel
		private: static dJointGroupID contactgroup;
		private: static dSpaceID car_space;
		private: static dGeomID box[1];
		private: static dGeomID sphere[3];
		private: static dGeomID ground;
		private: static dGeomID ground_box;

		private: dReal speed,steer;	//commands

		// private: static constexpr dVector3 yunit = { 0, 1, 0 };
		// private: static constexpr dVector3 zunit = { 0, 0, 1 };

		// private: dMatrix3 R;

		public: propagator(){
		}
		public: ~propagator(){
			ROS_INFO("propagator Done");
			//End ROS
			this->nh.shutdown();
			// End ODE
			dGeomDestroy(box[0]);
		  dGeomDestroy(sphere[0]);
		  dGeomDestroy(sphere[1]);
		  dGeomDestroy(sphere[2]);
		  dJointGroupDestroy(contactgroup);
		  dSpaceDestroy(dspace);
		  dWorldDestroy(dworld);
		  dCloseODE();
		}

		// this is called by dSpaceCollide when two objects in space are
		// potentially colliding.
		public: static void nearCallback(void *, dGeomID o1, dGeomID o2){
			int i,numc;

			// only collide things with the ground
			int g1 = (o1 == ground || o1 == ground_box);
			int g2 = (o2 == ground || o2 == ground_box);
			if (!(g1 ^ g2)) return;

			const int MAX_CONTACTS = 10;
			dContact contact[MAX_CONTACTS];
			numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact));
			if (numc > 0) {
				for (i=0; i<numc; i++) {
					contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	dContactSoftERP | dContactSoftCFM | dContactApprox1;
					// Surface friction coefficient
					contact[i].surface.mu = dInfinity;
					// Surface slip coefficient
					contact[i].surface.slip1 = 0.1;
					contact[i].surface.slip2 = 0.1;
					// Sofr ERP (Make surface soft)
					contact[i].surface.soft_erp = 0.5;
					// Soft CFM (Make surface soft)
					contact[i].surface.soft_cfm = 0.3;
					dJointID ct = dJointCreateContact (dworld,contactgroup,&contact[i]);
					dJointAttach (ct,
												dGeomGetBody(contact[i].geom.g1),
												dGeomGetBody(contact[i].geom.g2));
				}
			}
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)//Initialization
		{
			ROS_INFO("Load propagator!");

			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "propagator");
			//Subscriber
			planner2gazebo 	= nh.subscribe("/PlannerStates", 1, &propagator::P2G_cb, this);
			//Publisher
			gazeboCtrl 			= nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1, true);
			gazebo2planner	= nh.advertise<dynamo_planner::custom_states_msgs>("/GazeboStates", 1, true);
			//
			// Physics engine
			model = _parent;
			world = model->GetWorld();
			physics_engine = world->Physics();
			//Control initialization
			speed=0.,steer=0.;
			// create ODE world
			dInitODE2(0);
			dAllocateODEDataForThread(dAllocateMaskAll);
			dworld = dWorldCreate();
			dspace = dHashSpaceCreate(0);
			contactgroup = dJointGroupCreate(0);
			dWorldSetGravity(dworld,0,0,-9.81);
			ground = dCreatePlane(dspace,0,0,1,0);
			// Car geometry settings
			/************URDF 불러오던가 직접 포팅하던가***********/
			// ODE buggy
			int i;
			dMass mass;
			double st[3] = {5.0, 5.0, -M_PI};
			// constexpr dVector3 yunit = { 0, 1, 0 };
			// constexpr dVector3 zunit = { 0, 0, 1 };
			// chassis body
		  body[0] = dBodyCreate(dworld);
		  dBodySetPosition(body[0],st[0],st[1],STARTZ);
		  dMassSetBox(&mass,1,LENGTH,WIDTH,HEIGHT);
		  dMassAdjust(&mass,CMASS);
		  dBodySetMass(body[0],&mass);
		  box[0] = dCreateBox(0,LENGTH,WIDTH,HEIGHT);
		  dGeomSetBody(box[0],body[0]);
		  // wheel bodies
		  for (i=1; i<=3; i++) {
		    body[i] = dBodyCreate(dworld);
		    dQuaternion quart;
		    dQFromAxisAndAngle(quart,1,0,0,M_PI*0.5);
		    dBodySetQuaternion(body[i],quart);
		    dMassSetSphere(&mass,1,RADIUS);
		    dMassAdjust(&mass,WMASS);
		    dBodySetMass(body[i],&mass);
		    sphere[i-1] = dCreateSphere(0,RADIUS);
		    dGeomSetBody(sphere[i-1],body[i]);
		  }
		  dBodySetPosition(body[1],0.5*LENGTH+st[0],st[1],STARTZ-HEIGHT*0.5);
		  dBodySetPosition(body[2],-0.5*LENGTH+st[0], WIDTH*0.5+st[1],STARTZ-HEIGHT*0.5);
		  dBodySetPosition(body[3],-0.5*LENGTH+st[0],-WIDTH*0.5+st[1],STARTZ-HEIGHT*0.5);
		  // front and back wheel hinges
		  for (i=0; i<3; i++) {
		    joint[i] = dJointCreateHinge2(dworld,0);
		    dJointAttach(joint[i],body[0],body[i+1]);
		    const dReal *axis = dBodyGetPosition (body[i+1]);
		    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
		    // dJointSetHinge2Axes(joint[i], zunit, yunit);
				dJointSetHinge2Axis1(joint[i],0,0,1);
				dJointSetHinge2Axis2(joint[i],0,1,0);
		  }
		  // set joint suspension
		  for (i=0; i<3; i++) {
		    dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
		    dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
		  }
		  // lock back wheels along the steering axis
		  for (i=1; i<3; i++) {
		    // set stops to make sure wheels always stay in alignment
		    dJointSetHinge2Param (joint[i],dParamLoStop,0);
		    dJointSetHinge2Param (joint[i],dParamHiStop,0);
		  }
			/************************************************************************/
			// create car space and add it to the top level space
		  car_space = dSimpleSpaceCreate(dspace);
		  dSpaceSetCleanup(car_space,0);
		  dSpaceAdd(car_space,box[0]);
		  dSpaceAdd(car_space,sphere[0]);
		  dSpaceAdd(car_space,sphere[1]);
		  dSpaceAdd(car_space,sphere[2]);



			// std::cerr << "\nThe propagator plugin is attach to model[" <<	model->GetName() << "], world[" << world->GetName() <<"]\n";
			ROS_INFO("World name : %s", world->Name().c_str());
			ROS_INFO("Robot model : %s", model->GetName().c_str());
			ROS_INFO("Get model ID : %d", model->GetId());
			ROS_INFO("Get model GetChildCount : %d", model->GetChildCount());
			ROS_INFO("Physics engine : %s", physics_engine->GetType().c_str());
			ROS_INFO("Time step size : %f", physics_engine->GetMaxStepSize());//In case of ODE, it is time step size
			ROS_INFO("Update period : %f", physics_engine->GetUpdatePeriod());
			// Safety check
		  if (model->GetJointCount() == 0)
		  {
		    std::cerr << "Invalid joint count, plugin not loaded\n";
		    return;
		  }
			// gazebo Joint controller
			jcX = new physics::JointController(model);
			jcY = new physics::JointController(model);
			jcZ = new physics::JointController(model);
			jcYaw = new physics::JointController(model);
			// gazebo Joint position
			jX = model->GetJoints()[1];
			jY = model->GetJoints()[2];
			jZ = model->GetJoints()[3];
			jYaw = model->GetJoints()[4];
			// Joint name
			std::cout << "***Robot joint name***" << std::endl;
			std::cout << jX->GetName() << std::endl;
			std::cout << jY->GetName() << std::endl;
			std::cout << jZ->GetName() << std::endl;
			std::cout << jYaw->GetName() << std::endl;
			std::cout << "***********************" << std::endl;
			// Parameters
			time = 0;
			//Position
			posX = 0;
			posY = 0;
			posYAW = 0;
			//Control
			ctrlX = 0;
			ctrlY = 0;
			ctrlYAW = 0;
			// Propagated states
			PropStates.x = 0.0;
	    PropStates.y = 0.0;
	    PropStates.yaw = 0.0;
			PropStates.flag = false;
			//Spawn robot current position
			jcX->SetJointPosition(jX, posX);
			jcY->SetJointPosition(jY, posY);
			jcZ->SetJointPosition(jZ, 0.1651);//z-axis offset=Wheel radius
			jcYaw->SetJointPosition(jYaw, posYAW);

			// updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&propagator::OnUpdate, this, _1));
		}

		public: void P2G_cb(const dynamo_planner::custom_states_msgs& _msg){
			// std::cout << "CALL-BACK!!!" << std::endl;
			// Messege from planner
			PropStates.pre_x = _msg.x;
			PropStates.pre_y = _msg.y;
			PropStates.pre_yaw = _msg.yaw;
			double roll=0. ,pitch=0. ,yaw=_msg.yaw;
			dMatrix3 R;
			std::cout << "Messeges : " << _msg.x << ", " << _msg.y << ", "<< _msg.yaw <<std::endl;
			// physics_engine->SetMaxStepSize(_msg.duration);
			/********************* Physics engine RUN *****************/
			// Set vehicle pose/velocity
			// ODE buggy
			dBodySetPosition(body[0],_msg.x,_msg.y,STARTZ);
			dBodySetPosition(body[1],0.5*LENGTH+_msg.x,_msg.y,STARTZ-HEIGHT*0.5);
		  dBodySetPosition(body[2],-0.5*LENGTH+_msg.x, WIDTH*0.5+_msg.y,STARTZ-HEIGHT*0.5);
		  dBodySetPosition(body[3],-0.5*LENGTH+_msg.x,-WIDTH*0.5+_msg.y,STARTZ-HEIGHT*0.5);
			// dRFromAxisAndAngle(R,0,0,1,_msg.yaw);
			dRFromEulerAngles(R,roll,pitch,yaw);
			dBodySetRotation(body[0], R);
			dBodySetRotation(body[1], R);
			dBodySetRotation(body[2], R);
			dBodySetRotation(body[3], R);
			// // Husky
			// dBodySetPosition(body[0], _msg.x, _msg.y, 0.1651);
			// dRFromAxisAndAngle(R,0,0,1,_msg.yaw);
			// dBodySetRotation (body[0], R);
			// dBodySetLinearVel(body[0], _msg.controlX, _msg.controlY, 0.);
			// dBodySetAngularVel(body[0], 0., 0., _msg.controlYAW);
			//Control input
			speed = sqrt(pow(_msg.controlX,2)+pow(_msg.controlY,2));
			steer = _msg.controlYAW;
			// Acceleration
			dJointSetHinge2Param (joint[0],dParamVel2,-speed);
	    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
			// steering
			dReal v = steer - dJointGetHinge2Angle1(joint[0]);
			if (v > 0.1) v = 0.1;
			if (v < -0.1) v = -0.1;
			v *= 10.0;
			dJointSetHinge2Param(joint[0],dParamVel,v);
			dJointSetHinge2Param(joint[0],dParamFMax,0.2);
			dJointSetHinge2Param(joint[0],dParamLoStop,-0.75);
			dJointSetHinge2Param(joint[0],dParamHiStop,0.75);
			dJointSetHinge2Param(joint[0],dParamFudgeFactor,0.1);
			// Collision detection(Only check for the ground)
			dSpaceCollide(dspace,0,&nearCallback);
			//Precise version
			dWorldStep(dworld,0.001);
			// //Quick version
			// dWorldQuickStep(dworld,_msg.duration);
			// Get vehicle state
			const double* const dpos = dBodyGetPosition(body[0]);
			const double* const dquar = dBodyGetQuaternion(body[0]);
			const double* const dlinvel = dBodyGetLinearVel(body[0]);
			const double* const dangvel = dBodyGetAngularVel(body[0]);
			// Quarternion to Euler angle
			double quatx,quaty,quatz,quatw;
	    quatx = dquar[0];
	    quaty = dquar[1];
	    quatz = dquar[2];
	    quatw = dquar[3];
	    tf2::Quaternion quat(quatx,quaty,quatz,quatw);
	    tf2::Matrix3x3 mat(quat);
	    mat.getRPY(roll,pitch,yaw);

			posX = dpos[0];
			posY = dpos[1];
			posYAW = yaw;

			std::cout << "Get position : " << dpos[0] << ", " << dpos[1] << ", " << dpos[2] << std::endl;
			std::cout << "Get rotation : " << yaw << std::endl;
			std::cout << "Get lin_vel  : " << dlinvel[0] << ", " << dlinvel[1] << std::endl;
			std::cout << "Get ang_vel  : " << dangvel[2] << std::endl;
			// remove all contact joints
			dJointGroupEmpty(contactgroup);
			/********************* Physics engine END *****************/
			// gazeboCtrl.publish(vel);
			PropStates.x = posX;
			PropStates.y = posY;
			PropStates.yaw = posYAW;
			gazebo2planner.publish(PropStates);
		}

		public: void OnUpdate(const common::UpdateInfo &){

		}

	};
	dWorldID propagator::dworld;
	dSpaceID propagator::dspace;
	dBodyID propagator::body[4];
	dJointID propagator::joint[16];
	dJointGroupID propagator::contactgroup;
	dSpaceID propagator::car_space;
	dGeomID propagator::box[1];
	dGeomID propagator::sphere[3];
	dGeomID propagator::ground;
	dGeomID propagator::ground_box;
	// dReal propagator::speed,propagator::steer;
	// constexpr dVector3 propagator::yunit;
	// constexpr dVector3 propagator::zunit;

	GZ_REGISTER_MODEL_PLUGIN(propagator)
}
