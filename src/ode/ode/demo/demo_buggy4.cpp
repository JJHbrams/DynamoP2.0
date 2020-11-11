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


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <time.h>
#include <random>

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
#define STARTZ 0.7
#define HBASE 0.08
#define BELLYX 0.2155
#define BELLYZ 0.09
#define HIPX 0.227
#define HIPY 0.116
#define PROCX 0.0635
#define PROXY 0.009
#define HEATFINX 0.0635
#define HEATFINY 0.074
#define T_HEATFINY 0.035
#define T_HEATFINZ 0.125
#define KFE_ACTY 0.069
#define KFE_ACTZ  0.25
#define U_PROCY 0.005
#define SHANKX 0.065
#define SHANKY 0.015
#define SHANKZ 0.01
#define ADAPTERZ 0.160625
#define FOOTZ 0.02325

#define X 0
#define Y 1
#define Z 2

#define M_PI 3.14159265359


static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };


// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[40];
static dJointID joint[39];
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID robot_space;
static dGeomID box[12];
static dGeomID cylinder[24];
static dGeomID sphere[4];
static dGeomID ground_box;


// things that the user controls

dsFunctions fn;
static dReal speedA=0,speedB=0;	// user commands

const int NUM_PARTS = 40;
dReal ORI[NUM_PARTS][3];

double NUM_SAMPLES = 0;

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

// ANYmal quadrupeds
void ANYmal_def(){

  int i;
  dMass mass;
  dMatrix3 R;

  dReal LENGTH, WIDTH, HEIGHT, RADIUS;
  dReal MASS;

  dReal POS[NUM_PARTS][3];

  for (i=0; i<NUM_PARTS; i++){
    POS[i][X] = ORI[i][X];
    POS[i][Y] = ORI[i][Y];
    POS[i][Z] = ORI[i][Z];
  }

  // Main body
  LENGTH=0.531, WIDTH=0.27, HEIGHT=0.24;
  MASS=16.793507758;
  dBodySetPosition (body[0],POS[0][X],POS[0][Y],POS[0][Z]);
  dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&mass,MASS);
  dBodySetMass (body[0],&mass);
  dGeomSetBody (box[0],body[0]);
  // Belly
  LENGTH=0.1, WIDTH=0.1, HEIGHT=0.07;
  MASS=0.001;
  for(int BELLY=1;BELLY<3;BELLY++){
    dBodySetPosition (body[BELLY],POS[BELLY][X],POS[BELLY][Y],POS[BELLY][Z]);
    dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[BELLY],&mass);
    dGeomSetBody (box[BELLY],body[BELLY]);
  }
  LENGTH=0.531, WIDTH=0.02, HEIGHT=0.07;
  dBodySetPosition (body[3],POS[3][X],POS[3][Y],POS[3][Z]);
  dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&mass,MASS);
  dBodySetMass (body[3],&mass);
  dGeomSetBody (box[3],body[3]);
  // Legs
  int HIP1=1, HIP2=2, HIP3=3, THIGH1=4, THIGH2=5, THIGH3=6, SHANK=7, ADAPTER=8, FOOT=9;
  for (i=0; i<4; i++) {//LF RF LH RH
    /// HIP
    // HAA actuator
    LENGTH=0.1, RADIUS=0.05;
    MASS=1.42462064;
    dRFromEulerAngles(R,0,M_PI/2,0);
    dBodySetRotation(body[HIP1*4+i],R);
    dBodySetPosition (body[HIP1*4+i],POS[HIP1*4+i][X],POS[HIP1*4+i][Y],POS[HIP1*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[HIP1*4+i],&mass);
    dGeomSetBody (cylinder[i],body[HIP1*4+i]);
    // Protector
    LENGTH=0.1, RADIUS=0.08;
    MASS=0.001;
    dRFromEulerAngles(R,M_PI/2,0,0);
    dBodySetRotation(body[HIP2*4+i],R);
    dBodySetPosition (body[HIP2*4+i],POS[HIP2*4+i][X],POS[HIP2*4+i][Y],POS[HIP2*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[HIP2*4+i],&mass);
    dGeomSetBody (cylinder[i+4],body[HIP2*4+i]);
    // Heatfins
    LENGTH=0.03, RADIUS=0.045;
    MASS=0.001;
    dRFromEulerAngles(R,M_PI/2,0,0);
    dBodySetRotation(body[HIP3*4+i],R);
    dBodySetPosition (body[HIP3*4+i],POS[HIP3*4+i][X],POS[HIP3*4+i][Y],POS[HIP3*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[HIP3*4+i],&mass);
    dGeomSetBody (cylinder[i+8],body[HIP3*4+i]);

    /// Thigh
    // Thigh w/ heatfins
    LENGTH=0.08, WIDTH=0.04, HEIGHT=0.25;
    MASS=1.634976467;
    if(i%2 == 0)  dRFromEulerAngles(R,0.145,0,0);
    else          dRFromEulerAngles(R,-0.145,0,0);
    dBodySetRotation(body[THIGH1*4+i],R);
    dBodySetPosition (body[THIGH1*4+i],POS[THIGH1*4+i][X],POS[THIGH1*4+i][Y],POS[THIGH1*4+i][Z]);
    dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[THIGH1*4+i],&mass);
    dGeomSetBody (box[i+4],body[THIGH1*4+i]);
    // Upper protector
    LENGTH=0.12, RADIUS=0.066;
    MASS=0.001;
    if(i%2 == 0)  dRFromEulerAngles(R,1.71579632679,0,0);
    else          dRFromEulerAngles(R,-1.71579632679,0,0);
    dBodySetRotation(body[THIGH2*4+i],R);
    dBodySetPosition (body[THIGH2*4+i],POS[THIGH2*4+i][X],POS[THIGH2*4+i][Y],POS[THIGH2*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[THIGH2*4+i],&mass);
    dGeomSetBody (cylinder[i+12],body[THIGH2*4+i]);
    // KFE actuator
    LENGTH=0.12, RADIUS=0.06;
    MASS=0.001;
    dRFromEulerAngles(R,M_PI/2,0,0);
    dBodySetRotation(body[THIGH3*4+i],R);
    dBodySetPosition (body[THIGH3*4+i],POS[THIGH3*4+i][X],POS[THIGH3*4+i][Y],POS[THIGH3*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[THIGH3*4+i],&mass);
    dGeomSetBody (cylinder[i+16],body[THIGH3*4+i]);

    /// Shank
    LENGTH=0.08, WIDTH=0.07, HEIGHT=0.13;
    MASS=0.207204302;
    dRFromEulerAngles(R,0,M_PI/2,0);
    dBodySetRotation(body[SHANK*4+i],R);
    dBodySetPosition (body[SHANK*4+i],POS[SHANK*4+i][X],POS[SHANK*4+i][Y],POS[SHANK*4+i][Z]);
    dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[SHANK*4+i],&mass);
    dGeomSetBody (box[i+8],body[SHANK*4+i]);

    /// Adapter
    LENGTH=0.32125, RADIUS=0.015;
    MASS=0.140170767;
    dRFromEulerAngles(R,0,0,0);
    dBodySetRotation(body[ADAPTER*4+i],R);
    dBodySetPosition (body[ADAPTER*4+i],POS[ADAPTER*4+i][X],POS[ADAPTER*4+i][Y],POS[ADAPTER*4+i][Z]);
    dMassSetCylinder (&mass,1,3,RADIUS,LENGTH);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[ADAPTER*4+i],&mass);
    dGeomSetBody (cylinder[i+20],body[ADAPTER*4+i]);

    /// Foot
    RADIUS=0.031;
    MASS=0.001;
    dBodySetPosition (body[FOOT*4+i],POS[FOOT*4+i][X],POS[FOOT*4+i][Y],POS[FOOT*4+i][Z]);
    dMassSetSphere (&mass,1,RADIUS);
    dMassAdjust (&mass,MASS);
    dBodySetMass (body[FOOT*4+i-1],&mass);
    dGeomSetBody (sphere[i],body[FOOT*4+i]);
  }

  // Hip hinges
  for (i=0; i<15; i++) {
    dJointAttach(joint[i],body[0],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  // Thigh hinges
  for (i=15; i<23; i++) {
    dJointAttach(joint[i],body[HIP1*4+(i-3)%4],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  for (i=23; i<27; i++) {
    dJointAttach(joint[i],body[THIGH1*4+(i-3)%4],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+5]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  //shank
  for (i=27; i<31; i++) {
    dJointAttach(joint[i],body[THIGH3*4+(i-3)%4],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  //Adpater
  for (i=31; i<35; i++) {
    dJointAttach(joint[i],body[SHANK*4+(i-3)%4],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  //FOOT
  for (i=35; i<39; i++) {
    dJointAttach(joint[i],body[ADAPTER*4+(i-3)%4],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],0,1,0);
  }
  // lock back wheels along the steering axis
  for (i=0; i<39; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
  }
}

// start simulation - set viewpoint
static void start(){
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {1.0f,1.5f,1.0};
  static float hpr[3] = {-120.0000f,-20.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
	  "\t'z' to decrease speed.\n"
	  "\t',' to steer left.\n"
	  "\t'.' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}

// called when a key pressed
static void command (int cmd){
  switch (cmd) {
  case 'a': case 'A':
    speedA += 0.3;
    speedB += 0.3;
    break;
  case 'z': case 'Z':
    speedA -= 0.3;
    speedB -= 0.3;
    break;
  case ',':
    speedA -= 0.3;
    speedB += 0.3;
    break;
  case '.':
    speedA += 0.3;
    speedB -= 0.3;
    break;
  case ' ':
    speedA = 0.5;
    speedB = 0.5;
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

// simulation loop
std::string TR_FILE = "/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/data/TR_two_wheeled.txt";
std::string TE_FILE = "/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/data/TE_two_wheeled.txt";
std::fstream dataout(TR_FILE, std::ios::out);
static void simLoop (int pause){
  int i;
  dVector3 dpos,dpos2;
  dMatrix3 drot,drot2;
  dReal dyaw,dyaw2;
  dQuaternion dquar;//w x y z

  dReal X_set = RAND_VAL(-5,5,'u'), Y_set = RAND_VAL(-5,5,'u'), YAW_set = RAND_VAL(-M_PI,M_PI,'u');
  dReal speedA = RAND_VAL(-100.0,100.0,'n'), speedB = RAND_VAL(-100.0,100.0,'n');
  dReal TIME_set = RAND_VAL(100, 5000,'u');

  double SIM_TIME = 0.001;//0.581393388217187;
  double TIME_DENOM = 1000;

  // TIME_set = 0.581393388217187;

/*
  dMatrix3 R;
  dReal POS[4][3];
  for (i=0; i<=3; i++){
    POS[i][X] = (ORI[i][X]+X_set)*cos(YAW)-(ORI[i][Y]+Y_set)*sin(YAW);
    POS[i][Y] = (ORI[i][X]+X_set)*sin(YAW)+(ORI[i][Y]+Y_set)*cos(YAW);
    POS[i][Z] = ORI[i][Z];
  }
  dReal MV_AX, MV_AY;
  MV_AX = sgn(YAW)*((2/M_PI)*abs(YAW-sgn(YAW)*(M_PI/2))-1);
  MV_AY = -(2/M_PI)*abs(YAW)+1;
  dRFromAxisAndAngle(R,0,0,1,YAW);
  dBodySetRotation(body[0], R);
  dBodySetPosition (body[0],POS[0][X],POS[0][Y],POS[0][Z]);
  for (i=1; i<=3; i++) {
    dRFromEulerAngles(R,M_PI/2,YAW,0);
    dBodySetRotation(body[i],R);
    dBodySetPosition (body[i],POS[i][X],POS[i][Y],POS[i][Z]);
  }

  dJointAttach(joint[i],body[0],body[i+1]);
  const dReal *axis = dBodyGetPosition(body[i+1]);
  dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
  // dJointSetHinge2Axes (joint[i], zunit, yunit);
  dJointSetHinge2Axis1(joint[i],0,0,1);
  dJointSetHinge2Axis2(joint[i],MV_AX,MV_AY,0);

*/
  // Generate data
  double NUM_TR = 300000;
  double NUM_TE = 5000;
  if(NUM_SAMPLES >= NUM_TR) {
    dataout.close();
    return;
  }
  ANYmal_def();
/*
  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,0,0);
  for (i=1; i<=2; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
  dsSetColor (1,1,1);
  for (i=3; i<=4; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
*/
/*
  dBodyCopyPosition(body[0],dpos);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot);
  dyaw = atan2(drot[4],drot[0]);

  std::cout << "IN  : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << ", " << speedA << ", " << speedB << ", " << TIME_set << std::endl;
*/
  for(int cnt=0;cnt<1;cnt++) {

    if (!pause) {
      /*
      // motor
			dJointSetHinge2Param (joint[0],dParamVel2,-speedA);
	    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[1],dParamVel2,-speedB);
	    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[2],dParamVel2,-speedA);
	    dJointSetHinge2Param (joint[2],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[3],dParamVel2,-speedB);
	    dJointSetHinge2Param (joint[3],dParamFMax2,0.1);

      */

      dSpaceCollide (space,0,&nearCallback);
      dWorldQuickStep (world,SIM_TIME);
/*
      dsSetColor (1,1,0);
      dsSetTexture (DS_WOOD);
      dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
      dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
      dsSetColor (1,0,0);
      for (i=1; i<=2; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
      dsSetColor (1,1,1);
      for (i=3; i<=4; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
*/
    }

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }
  /*
  dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),LENGTH,WIDTH,HEIGHT);
  dsSetColor (1,0,0);
  for (i=1; i<=2; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),LENGTH,RADIUS);
  dsSetColor (1,1,1);
  for (i=3; i<=4; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),LENGTH,RADIUS);
*/

  dReal LENGTH, WIDTH, HEIGHT, RADIUS;
  dReal sides[3];
  dsSetTexture (DS_WOOD);
  // Main body
  dsSetColor (0.1,0.1,0);
  LENGTH=0.531, WIDTH=0.27, HEIGHT=0.24; sides[0] = LENGTH, sides[1] = WIDTH, sides[2] = HEIGHT;
  dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  // Belly
  dsSetColor (0,1,0);
  LENGTH=0.1, WIDTH=0.1, HEIGHT=0.07; sides[0] = LENGTH, sides[1] = WIDTH, sides[2] = HEIGHT;
  for(i=1;i<3;i++)  dsDrawBox(dBodyGetPosition(body[i]),dBodyGetRotation(body[i]),sides);
  LENGTH=0.531, WIDTH=0.02, HEIGHT=0.07; sides[0] = LENGTH, sides[1] = WIDTH, sides[2] = HEIGHT;
  dsDrawBox(dBodyGetPosition(body[3]),dBodyGetRotation(body[3]),sides);
  // Thigh w/ heatfins
  LENGTH=0.08, WIDTH=0.04, HEIGHT=0.25; sides[0] = LENGTH, sides[1] = WIDTH, sides[2] = HEIGHT;
  for(i=4; i<8; i++)  dsDrawBox(dBodyGetPosition(body[i+12]),dBodyGetRotation(body[i+12]),sides);
  // Shank
  LENGTH=0.08, WIDTH=0.07, HEIGHT=0.13; sides[0] = LENGTH, sides[1] = WIDTH, sides[2] = HEIGHT;
  for(i=8; i<12; i++) dsDrawBox(dBodyGetPosition(body[i+20]),dBodyGetRotation(body[i+20]),sides);

  //////////////////// Cylinder ///////////////////
  dsSetColor (0.5,0.5,0.5);
  // HAA actuator
  LENGTH=0.1, RADIUS=0.05;
  for (i=0; i<4; i++) dsDrawCylinder(dBodyGetPosition(body[i+4]), dBodyGetRotation(body[i+4]),LENGTH,RADIUS);
  // Protector
  LENGTH=0.1, RADIUS=0.08;
  for (i=4; i<8; i++) dsDrawCylinder(dBodyGetPosition(body[i+4]), dBodyGetRotation(body[i+4]),LENGTH,RADIUS);
  // Heatfins
  LENGTH=0.03, RADIUS=0.045;
  for (i=8; i<12; i++) dsDrawCylinder(dBodyGetPosition(body[i+4]), dBodyGetRotation(body[i+4]),LENGTH,RADIUS);
  // Upper protector
  LENGTH=0.12, RADIUS=0.066;
  for (i=12; i<16; i++) dsDrawCylinder(dBodyGetPosition(body[i+8]), dBodyGetRotation(body[i+8]),LENGTH,RADIUS);
  // KFE actuator
  LENGTH=0.12, RADIUS=0.06;
  for (i=16; i<20; i++) dsDrawCylinder(dBodyGetPosition(body[i+8]), dBodyGetRotation(body[i+8]),LENGTH,RADIUS);
  // Adapter
  LENGTH=0.32125, RADIUS=0.015;
  for (i=20; i<24; i++) dsDrawCylinder(dBodyGetPosition(body[i+12]), dBodyGetRotation(body[i+12]),LENGTH,RADIUS);

  //////////////////// Sphere ///////////////////
  dsSetColor (1,0,0);
  // Foot
  RADIUS=0.031;
  for (i=0; i<4; i++) dsDrawSphere(dBodyGetPosition(body[i+36]), dBodyGetRotation(body[i+36]),RADIUS);


/*
  dBodyCopyPosition(body[0],dpos2);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot2);
  dyaw2 = atan2(drot2[4],drot2[0]);
  std::cout << "OUT : " << dpos2[0] << ", " << dpos2[1] << ", " << dyaw2/M_PI*180 << std::endl;

  if(dataout.is_open()){
    if(NUM_SAMPLES == 0)
    dataout << "         " << ", " << "Xa"    << ","  << "Ya"    << ", " << "YAWa"<< ", " << "CTRLA"<< ", " << "CTRLB"<< ", " << "Duration"    << ", " <<"Xb"      << ", " << "Yb"     << ", " << "YAWb"<<"\n";
    dataout << NUM_SAMPLES << ", " << dpos[0] << ", " << dpos[1] << ", " << dyaw  << ", " << speedA << ", " << speedB << ", " << TIME_set*SIM_TIME << ", " << dpos2[0] << ", " << dpos2[1] << ", " << dyaw2 << "\n";
  }

  NUM_SAMPLES++;*/
  return;
}

void init_ODE(){
  int i;
  dReal YAW = 0.0;
  dReal BASE_H = HBASE+STARTZ;
  // setup pointers to drawstuff callback functions
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // Base
  ORI[0][X] = 0.0;       ORI[0][Y] = 0.0;         ORI[0][Z] = BASE_H;
  // Belly plate bump front hind middle
  ORI[1][X] = BELLYX;       ORI[1][Y] = 0.0;         ORI[1][Z] = BASE_H-BELLYZ;
  ORI[2][X] = -BELLYX;       ORI[2][Y] = 0.0;         ORI[2][Z] = BASE_H-BELLYZ;
  ORI[3][X] = 0.0;       ORI[3][Y] = 0.0;         ORI[3][Z] = BASE_H-BELLYZ;
  // LF RF LH RH

  // Hip
  // HAA actuators
  ORI[4][X] = HIPX+0.277;   ORI[4][Y] = HIPY;   ORI[4][Z] = BASE_H;
  ORI[5][X] = HIPX+0.277;   ORI[5][Y] = -HIPY;  ORI[5][Z] = BASE_H;
  ORI[6][X] = -HIPX-0.277;  ORI[6][Y] = HIPY;   ORI[6][Z] = BASE_H;
  ORI[7][X] = -HIPX-0.277;  ORI[7][Y] = -HIPY;  ORI[7][Z] = BASE_H;
  // Protector
  ORI[8][X] = PROCX;       ORI[8][Y] = -PROCX;         ORI[8][Z] = BASE_H;
  ORI[9][X] = PROCX;       ORI[9][Y] = PROCX;         ORI[9][Z] = BASE_H;
  ORI[10][X] = -PROCX;       ORI[10][Y] = -PROCX;         ORI[10][Z] = BASE_H;
  ORI[11][X] = -PROCX;       ORI[11][Y] = PROCX;         ORI[11][Z] = BASE_H;
  // Heat fins
  ORI[12][X] = HEATFINX;       ORI[12][Y] = -HEATFINY;         ORI[12][Z] = BASE_H;
  ORI[13][X] = HEATFINX;       ORI[13][Y] = HEATFINY;         ORI[13][Z] = BASE_H;
  ORI[14][X] = -HEATFINX;       ORI[14][Y] = -HEATFINY;         ORI[14][Z] = BASE_H;
  ORI[15][X] = -HEATFINX;       ORI[15][Y] = HEATFINY;         ORI[15][Z] = BASE_H;

  // Thigh
  // thigh with heat fins
  ORI[16][X] = HIPX;       ORI[16][Y] = HIPY+T_HEATFINY;         ORI[16][Z] = BASE_H-T_HEATFINZ;
  ORI[17][X] = HIPX;       ORI[17][Y] = -HIPY-T_HEATFINY;         ORI[17][Z] = BASE_H-T_HEATFINZ;
  ORI[18][X] = -HIPX;       ORI[18][Y] = HIPY+T_HEATFINY;         ORI[18][Z] = BASE_H-T_HEATFINZ;
  ORI[19][X] = -HIPX;       ORI[19][Y] = -HIPY-T_HEATFINY;         ORI[19][Z] = BASE_H-T_HEATFINZ;
  // KFE actuator
  ORI[20][X] = HIPX;       ORI[20][Y] = HIPY+KFE_ACTY;         ORI[20][Z] = BASE_H-KFE_ACTZ;
  ORI[21][X] = HIPX;       ORI[21][Y] = -HIPY-KFE_ACTY;         ORI[21][Z] = BASE_H-KFE_ACTZ;
  ORI[22][X] = -HIPX;       ORI[22][Y] = HIPY+KFE_ACTY;         ORI[22][Z] = BASE_H-KFE_ACTZ;
  ORI[23][X] = -HIPX;       ORI[23][Y] = -HIPY-KFE_ACTY;         ORI[23][Z] = BASE_H-KFE_ACTZ;
  // upper protector
  ORI[24][X] = HIPX;       ORI[24][Y] = HIPY-U_PROCY;         ORI[24][Z] = BASE_H;
  ORI[25][X] = HIPX;       ORI[25][Y] = -HIPY+U_PROCY;         ORI[25][Z] = BASE_H;
  ORI[26][X] = -HIPX;       ORI[26][Y] = HIPY-U_PROCY;         ORI[26][Z] = BASE_H;
  ORI[27][X] = -HIPX;       ORI[27][Y] = -HIPY+U_PROCY;         ORI[27][Z] = BASE_H;

  // Shank
  ORI[28][X] = HIPX+SHANKX;       ORI[28][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[28][Z] = BASE_H-KFE_ACTZ+SHANKZ;
  ORI[29][X] = HIPX+SHANKX;       ORI[29][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[29][Z] = BASE_H-KFE_ACTZ+SHANKZ;
  ORI[30][X] = -HIPX-SHANKX;       ORI[30][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[30][Z] = BASE_H-KFE_ACTZ+SHANKZ;
  ORI[31][X] = -HIPX-SHANKX;       ORI[31][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[31][Z] = BASE_H-KFE_ACTZ+SHANKZ;

  // Adpater
  ORI[32][X] = HIPX+SHANKX;       ORI[32][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[32][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ;
  ORI[33][X] = HIPX+SHANKX;       ORI[33][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[33][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ;
  ORI[34][X] = -HIPX-SHANKX;       ORI[34][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[34][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ;
  ORI[35][X] = -HIPX-SHANKX;       ORI[35][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[35][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ;

  // Foot
  ORI[36][X] = HIPX+SHANKX;       ORI[36][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[36][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ-0.32125+FOOTZ;
  ORI[37][X] = HIPX+SHANKX;       ORI[37][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[37][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ-0.32125+FOOTZ;
  ORI[38][X] = -HIPX-SHANKX;       ORI[38][Y] = HIPY+KFE_ACTY-SHANKY;         ORI[38][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ-0.32125+FOOTZ;
  ORI[39][X] = -HIPX-SHANKX;       ORI[39][Y] = -HIPY-KFE_ACTY+SHANKY;         ORI[39][Z] = BASE_H-KFE_ACTZ+SHANKZ-ADAPTERZ-0.32125+FOOTZ;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-9.81);
  ground = dCreatePlane (space,0,0,1,0);

  dReal LENGTH, WIDTH, HEIGHT, RADIUS;
  //////////////////// BOX ///////////////////
  // Main body
  LENGTH=0.531, WIDTH=0.27, HEIGHT=0.24;
  body[0] = dBodyCreate (world);
  box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  for(int i=1;i<3;i++){
    LENGTH=0.1, WIDTH=0.1, HEIGHT=0.07;
    body[i] = dBodyCreate (world);
    box[i] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  }
  LENGTH=0.531, WIDTH=0.02, HEIGHT=0.07;
  body[3] = dBodyCreate (world);
  box[3] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  // Thigh w/ heatfins
  LENGTH=0.08, WIDTH=0.04, HEIGHT=0.25;
  for (i=4; i<8; i++) {
    body[i+12] = dBodyCreate (world);
    box[i] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  }
  // Shank
  LENGTH=0.08, WIDTH=0.07, HEIGHT=0.13;
  for (i=8; i<12; i++) {
    body[i+20] = dBodyCreate (world);
    box[i] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  }

  //////////////////// Cylinder ///////////////////
  // HAA actuator
  LENGTH=0.1, RADIUS=0.05;
  for (i=0; i<4; i++) {
    body[i+4] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }
  // Protector
  LENGTH=0.1, RADIUS=0.08;
  for (i=4; i<8; i++) {
    body[i+4] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }
  // Heatfins
  LENGTH=0.03, RADIUS=0.045;
  for (i=8; i<12; i++) {
    body[i+4] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }
  // Upper protector
  LENGTH=0.12, RADIUS=0.066;
  for (i=12; i<16; i++) {
    body[i+8] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }
  // KFE actuator
  LENGTH=0.12, RADIUS=0.06;
  for (i=16; i<20; i++) {
    body[i+8] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }
  // Adapter
  LENGTH=0.32125, RADIUS=0.015;
  for (i=20; i<24; i++) {
    body[i+12] = dBodyCreate (world);
    cylinder[i] = dCreateCylinder(0,RADIUS,LENGTH);
  }

  //////////////////// Sphere ///////////////////
  // Foot
  RADIUS=0.015;
  for (i=0; i<4; i++) {
    body[i+36] = dBodyCreate (world);
    sphere[i] = dCreateSphere(0,RADIUS);
  }

  //////////////////// Joints ///////////////////
  for (i=0; i<39; i++) {
    joint[i] = dJointCreateHinge2(world,0);
  }

  ANYmal_def();

  // create car space and add it to the top level space
  robot_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (robot_space,0);
  for (i=0; i<12; i++)  dSpaceAdd (robot_space,box[i]);
  for (i=0; i<24; i++)  dSpaceAdd (robot_space,cylinder[i]);
  for (i=0; i<4 ; i++)  dSpaceAdd (robot_space,sphere[i]);

  /*  // environment
  ground_box = dCreateBox (space,2,1.5,1);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,2,0,-0.34);
  dGeomSetRotation (ground_box,R);*/
}

int main (int argc, char **argv){
  int i;
  // Init world
  init_ODE();
  // run simulation
  dsSimulationLoop (argc,argv,768,512,&fn);

  /*while(1){
    int i;
    dVector3 dpos;
    dMatrix3 drot;
    dReal dyaw;
    dQuaternion dquar;//w x y z
    dMatrix3 R;

    srand((int)time(NULL));
    dReal X_set = (rand()%200-100)/100, Y_set = (rand()%200-100)/100, YAW = (rand()%628-314)/100, DELTA = 0.0;

    vehicle_def(X_set,Y_set,M_PI/6);

    dBodyCopyPosition(body[0],dpos);
    dBodyCopyRotation(body[0],drot);
    dyaw = atan2(drot[4],drot[0]);
    std::cout << "A.Get pose : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << std::endl;

    for(int cnt=0;cnt<1000;cnt++) {
      // motor
      dJointSetHinge2Param (joint[1],dParamVel2,-speedA);
      dJointSetHinge2Param (joint[1],dParamFMax2,0.1);
      dJointSetHinge2Param (joint[2],dParamVel2,-speedB);
      dJointSetHinge2Param (joint[2],dParamFMax2,0.1);

      dSpaceCollide (space,0,&nearCallback);
      dWorldStep (world,0.001);//1ms

      // remove all contact joints
      dJointGroupEmpty (contactgroup);
    }
    dBodyCopyPosition(body[0],dpos);
    dBodyCopyRotation(body[0],drot);
    dyaw = atan2(drot[4],drot[0]);
    std::cout << "B.Get pose : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << time(NULL) << std::endl;
  }*/
  for (i=0; i<12; i++)  dGeomDestroy (box[i]);
  for (i=0; i<24; i++)  dGeomDestroy (cylinder[i]);
  for (i=0; i<4 ; i++)  dGeomDestroy (sphere[i]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
