/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

buggy with suspension.
this also shows you how to use geom groups.

*/

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
// msgs
#include "dynamo_planner/custom_states_msgs.h"
#include <geometry_msgs/Twist.h>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen_conversions/eigen_msg.h>
// ODE
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "../../ode/ode/demo/texturepath.h"
// C++
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <time.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define LENGTH 0.98740000	// chassis length
#define WIDTH 0.57090000	// chassis width
#define HEIGHT 0.24750000	// chassis height
#define RADIUS 0.1651	// wheel radius
#define STARTZ 0.25	// starting height of chassis
#define CMASS 4.6034		// chassis mass
#define WMASS 0.2637	// wheel mass
#define WLENGTH 0.1143
#define WOFFSET 0.03282
#define WBASE 0.5120
#define TRACK 0.5708

#define X 0
#define Y 1
#define Z 2

#define M_PI 3.14159265359

dynamo_planner::custom_states_msgs PropStates;

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };
double start[3] = {0.0, 0.0, 0.0};

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;
static dBodyID body[5];
static dJointID joint[4];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[4];
static dGeomID ground_box;


// things that the user controls
static dReal speed=0,steer=0;	// user commands

dReal ORI[5][3];

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *, dGeomID o1, dGeomID o2){
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}
int sgn(dReal Val){
  if(Val==0)  return 0;
  else        return ((Val > 0) - (Val < 0));
}
dReal MV_AX(dReal YAW){
  return sgn(YAW)*((2/M_PI)*abs(YAW-sgn(YAW)*(M_PI/2))-1);
}
dReal MV_AY(dReal YAW){
  return -(2/M_PI)*abs(YAW)+1;
}
void vehicle_def(dReal X_POS, dReal Y_POS, dReal YAW){

  int i;
  dMass mass;
  dMatrix3 R;
  dReal POS[5][3];

  for (i=0; i<=4; i++){
    POS[i][X] = (ORI[i][X])*cos(YAW)-(ORI[i][Y])*sin(YAW);
    POS[i][Y] = (ORI[i][X])*sin(YAW)+(ORI[i][Y])*cos(YAW);
    POS[i][Z] = ORI[i][Z];
  }

  // chassis body
  dRFromAxisAndAngle(R,0,0,1,YAW);
  dBodySetRotation(body[0], R);
  dBodySetPosition (body[0],X_POS,Y_POS,ORI[0][Z]);
  dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&mass,CMASS);
  dBodySetMass (body[0],&mass);
  dGeomSetBody (box[0],body[0]);
  // wheel bodies
  for (i=1; i<=4; i++) {
    dRFromEulerAngles(R,M_PI/2,YAW,0);
    dBodySetRotation(body[i],R);

    dMassSetSphere (&mass,1,RADIUS);
    dMassAdjust (&mass,WMASS);
    dBodySetMass (body[i],&mass);
    dGeomSetBody (sphere[i-1],body[i]);
    dBodySetPosition (body[i],POS[i][X]+X_POS,POS[i][Y]+Y_POS,POS[i][Z]);
  }
  // front and back wheel hinges
  for (i=0; i<4; i++) {
    dJointAttach(joint[i],body[0],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],MV_AX(YAW),MV_AY(YAW),0);
  }
  // set joint suspension
  for (i=0; i<4; i++) {
    dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
  }
  // lock back wheels along the steering axis
  for (i=0; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
  }
}
// start simulation - set viewpoint
/*
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
	  "\t'z' to decrease speed.\n"
	  "\t',' to steer left.\n"
	  "\t'.' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}


// called when a key pressed

static void command (int cmd)
{
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case ',':
    steer -= 0.5;
    break;
  case '.':
    steer += 0.5;
    break;
  case ' ':
    speed = 3.3;
    steer = 0;
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}


// simulation loop

static void simLoop (int pause)
{
  int i;
  for(int cnt=0;cnt<1000;cnt++) {
    if (!pause) {
      // motor
      dJointSetHinge2Param (joint[0],dParamVel2,-speed);
      dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

      // steering
      dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
      if (v > 0.1) v = 0.1;
      if (v < -0.1) v = -0.1;
      v *= 10.0;
      dJointSetHinge2Param (joint[0],dParamVel,v);
      dJointSetHinge2Param (joint[0],dParamFMax,0.2);
      dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
      dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
      dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

      dSpaceCollide (space,0,&nearCallback);
      dWorldStep (world,0.001);//1ms

    }


    // remove all contact joints
    dJointGroupEmpty (contactgroup);


    dsSetColor (0,1,1);
    dsSetTexture (DS_WOOD);
    dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
    dsSetColor (1,1,1);
    for (i=1; i<=3; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
  				       dBodyGetRotation(body[i]),0.02f,RADIUS);
    //
    // dVector3 ss;
    // dGeomBoxGetLengths (ground_box,ss);
    // dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);

  }
  const double* const dpos = dBodyGetPosition(body[0]);
  std::cout << "Get position : " << dpos[0] << ", " << dpos[1] << ", " << dpos[2] << ", Time : " << time(NULL) << std::endl;
  return;

}*/
void init_ODE(){
  int i;
  dMass m;
  dReal X_init = start[0];
  dReal Y_init = start[1];
  dReal YAW_init = start[2];
  dReal DELTA_init = 0.0;

  ORI[0][X] = 0.0;          ORI[0][Y] = 0.0;         ORI[0][Z] = STARTZ;
  ORI[1][X] = WBASE/2;   ORI[1][Y] = TRACK*0.5;   ORI[1][Z] = STARTZ-HEIGHT/2+WOFFSET;
  ORI[2][X] = WBASE/2;   ORI[2][Y] = -TRACK*0.5;  ORI[2][Z] = STARTZ-HEIGHT/2+WOFFSET;
  ORI[3][X] = -WBASE/2;  ORI[3][Y] = TRACK*0.5;   ORI[3][Z] = STARTZ-HEIGHT/2+WOFFSET;
  ORI[4][X] = -WBASE/2;  ORI[4][Y] = -TRACK*0.5;  ORI[4][Z] = STARTZ-HEIGHT/2+WOFFSET;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-9.81);
  ground = dCreatePlane (space,0,0,1,0);

  body[0] = dBodyCreate (world);
  box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  for (i=1; i<=4; i++) {
    body[i] = dBodyCreate (world);
    sphere[i-1] = dCreateSphere (0,RADIUS);
    joint[i-1] = dJointCreateHinge2(world,0);
  }
  vehicle_def(X_init,Y_init,YAW_init);

  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);
  dSpaceAdd (car_space,box[0]);
  dSpaceAdd (car_space,sphere[0]);
  dSpaceAdd (car_space,sphere[1]);
  dSpaceAdd (car_space,sphere[2]);
  dSpaceAdd (car_space,sphere[3]);

  /*  // environment
  ground_box = dCreateBox (space,2,1.5,1);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,2,0,-0.34);
  dGeomSetRotation (ground_box,R);*/

  return;
}
void prop_cb(const dynamo_planner::custom_states_msgs& _msg){
  ros::Time begin = ros::Time::now();
  double posX, posY, posYAW;
  double ctrlX, ctrlY;
  double ctrlA, ctrlB;
  double time;

  dVector3 dpos;
  dMatrix3 drot;
  // Initialize States
  time = _msg.duration;
  // ctrl = sqrt(pow(_msg.controlX,2)+pow(_msg.controlY,2));
  ctrlA = _msg.controlA;
  ctrlB = _msg.controlB;

  double roll=0. ,pitch=0. ,yaw=_msg.yaw, delta=steer;
  dMatrix3 R;

  // std::cout << "Messeges    : X=" << _msg.x << ", Y=" << _msg.y << ", YAW="<< _msg.yaw*180/M_PI << std::endl;
  // std::cout << "Messeges : " << _msg.duration << std::endl;
  if(time <= 0) time = 0.5;

  /********************* Physics engine RUN *****************/
  // ODE buggy
  vehicle_def(_msg.x, _msg.y, _msg.yaw);
  // vehicle_def(0,0,M_PI/6,0);

  // speed = ctrl;
  // steer = ctrlYAW;//sgn(ctrlYAW)*(abs(ctrlYAW-1.5)-0.7);

  // Get position
  dBodyCopyPosition(body[0],dpos);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot);
  yaw = atan2(drot[4],drot[0]);
  // std::cout << "A.Get pose  : X=" << dpos[0] << ", Y=" << dpos[1] << ", YAW=" << yaw/M_PI*180 << std::endl;

  for(int cnt=0;cnt<1000;cnt++) {
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-ctrlA);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    dJointSetHinge2Param (joint[1],dParamVel2,-ctrlB);
    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);

    dJointSetHinge2Param (joint[2],dParamVel2,-ctrlA);
    dJointSetHinge2Param (joint[2],dParamFMax2,0.1);

    dJointSetHinge2Param (joint[3],dParamVel2,-ctrlB);
    dJointSetHinge2Param (joint[3],dParamFMax2,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldQuickStep (world,time/1000);//1ms

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }
  //Get velocity
  const double* const dlinvel = dBodyGetLinearVel(body[0]);
  const double* const dangvel = dBodyGetAngularVel(body[0]);
  // Get position
  dBodyCopyPosition(body[0],dpos);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot);
  yaw = atan2(drot[4],drot[0]);
  /*// Quarternion to Euler angle
  dQuaternion dquar;//w x y z
  dBodyCopyQuaternion(body[0], dquar);
  double qw=dquar[0], qx=dquar[1], qy=dquar[2], qz=dquar[3];
  tf2::Quaternion quat(qx,qy,qz,qw);//x y z w
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll,pitch,yaw);*/
  // Pose
  posX = dpos[0];
  posY = dpos[1];
  posYAW = yaw;
  // std::cout << "B.Get pose  : X=" << dpos[0] << ", Y=" << dpos[1] << ", YAW=" << yaw/M_PI*180 << std::endl;
  /********************* Physics engine END *****************/
  // Update States
  PropStates.pre_x = _msg.x;
  PropStates.pre_y = _msg.y;
  PropStates.pre_yaw = _msg.yaw;
  PropStates.duration = _msg.duration;
  PropStates.controlA = _msg.controlA;
  PropStates.controlB = _msg.controlB;
  PropStates.x = posX;
  PropStates.y = posY;
  PropStates.yaw = posYAW;
  PropStates.flag=true;
  ros::Time end = ros::Time::now();
  double propagation_time = end.toSec() - begin.toSec();
  // std::cout << "INPUT  : " << PropStates.pre_x << ", " << PropStates.pre_y << ", " << PropStates.pre_yaw << ", " << PropStates.controlA << ", " << PropStates.controlB << std::endl;
  // std::cout << "OUTPUT : " << PropStates.x << ", " << PropStates.y << ", " << PropStates.yaw << std::endl;
  // std::cout << "TIME   : " << propagation_time*1000 << " ms" << std::endl;
  // std::cout << PropStates << std::endl;
}

int main (int argc, char **argv)
{
  /*
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
  */

  ROS_INFO("Load propagator_ODE!");
  ros::init(argc, argv, "propagator_ODE");

  ros::NodeHandle nh("~");
  ros::Publisher ode2planner;
  ros::Subscriber planner2ode;

  //Publisher
  ode2planner	= nh.advertise<dynamo_planner::custom_states_msgs>("/Prop_States", 1, true);
  //Subscriber
  planner2ode = nh.subscribe("/PlannerStates", 1, &prop_cb);

  init_ODE();

  // run simulation
  // dsSimulationLoop (argc,argv,352,288,&fn);
  // ros::Rate rate(13000);
  while(ros::ok()){
    ros::spinOnce();

    if(PropStates.flag == true){
      ode2planner.publish(PropStates);
      PropStates.flag = false;
    }
    // rate.sleep();
  }

  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dGeomDestroy (sphere[2]);
  dGeomDestroy (sphere[3]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();

  ros::Duration(1.0).sleep();
  return 0;
}
