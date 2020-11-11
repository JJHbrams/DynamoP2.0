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

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };


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

dsFunctions fn;
static dReal speedA=0,speedB=0;	// user commands

dReal ORI[5][3];

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
  // body[0] = dBodyCreate (world);
  dRFromAxisAndAngle(R,0,0,1,YAW);
  dBodySetRotation(body[0], R);
  dBodySetPosition (body[0],X_POS,Y_POS,POS[0][Z]);
  dMassSetBox (&mass,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&mass,CMASS);
  dBodySetMass (body[0],&mass);
  // box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);


  // wheel bodies
  for (i=1; i<=4; i++) {
    // body[i] = dBodyCreate (world);
    // dQuaternion q;
    // dQFromAxisAndAngle (q,1,0,0,M_PI/2);
    // dBodySetQuaternion (body[i],q);

    dRFromEulerAngles(R,M_PI/2,YAW,0);
    dBodySetRotation(body[i],R);

    dMassSetSphere (&mass,1,RADIUS);
    dMassAdjust (&mass,WMASS);
    dBodySetMass (body[i],&mass);
    // sphere[i-1] = dCreateSphere (0,RADIUS);
    dGeomSetBody (sphere[i-1],body[i]);
    dBodySetPosition (body[i],POS[i][X]+X_POS,POS[i][Y]+Y_POS,POS[i][Z]);
  }

  // front and back wheel hinges
  for (i=0; i<4; i++) {
    // joint[i] = dJointCreateHinge2(world,0);
    dJointAttach(joint[i],body[0],body[i+1]);
    const dReal *axis = dBodyGetPosition(body[i+1]);
    dJointSetHinge2Anchor(joint[i],axis[0],axis[1],axis[2]);
    // dJointSetHinge2Axes (joint[i], zunit, yunit);
    dJointSetHinge2Axis1(joint[i],0,0,1);
    dJointSetHinge2Axis2(joint[i],MV_AX(YAW),MV_AY(YAW),0);
  }
  //
  // // set joint suspension
  // for (i=0; i<4; i++) {
  //   dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
  //   dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
  // }

  // lock back wheels along the steering axis
  for (i=0; i<4; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
    // the following alternative method is no good as the wheels may get out
    // of alignment:
    //   dJointSetHinge2Param (joint[i],dParamVel,0);
    //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
  }
}

// start simulation - set viewpoint
static void start(){
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.0f,-0.0f,5.8000f};
  static float hpr[3] = {90.0000f,-90.0000f,0.0000f};
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

dReal RAND_VAL(dReal MIN, dReal MAX){
  srand(rand()*time(NULL));
  dReal RES = 1E3;
  int DIV = static_cast<int>((MAX-MIN+1)*RES);
  dReal MEAN = ((MAX-MIN)/2.)*RES;

  return (rand()%DIV - MEAN)/RES;
}

// simulation loop
static void simLoop (int pause){
  int i;
  dVector3 dpos;
  dMatrix3 drot;
  dReal dyaw;
  dQuaternion dquar;//w x y z

  dReal X_set = RAND_VAL(-20,20), Y_set = RAND_VAL(-20,20), YAW = RAND_VAL(-M_PI,M_PI), DELTA = -3.0;
  dReal speedA = RAND_VAL(-10.0,10.0), speedB = RAND_VAL(-10.0,10.0);

  double TIME = 0.581393;
  double TIME_DENOM = 1000;

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
  vehicle_def(X_set,Y_set,YAW);

  // dsSetColor (0,1,1);
  // dsSetTexture (DS_WOOD);
  // dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  // dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  // dsSetColor (1,0,0);
  // dsDrawCylinder(dBodyGetPosition(body[1]), dBodyGetRotation(body[1]),0.02f,RADIUS);
  // dsSetColor (1,1,1);
  // for (i=2; i<=3; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
  dBodyCopyPosition(body[0],dpos);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot);
  dyaw = atan2(drot[4],drot[0]);
  std::cout << "IN  : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << ", " << speedA << ", " << speedB << std::endl;

  for(int cnt=0;cnt<TIME_DENOM;cnt++) {

    if (!pause) {
      // dMatrix3 R;
      // dRFromEulerAngles(R,0,0,0.0);
      // dBodySetRotation(body[0], R);

      // dQuaternion qurt;
      // dQFromAxisAndAngle (qurt,0,0,1,0.0);
      // dBodySetQuaternion (body[0],qurt);
      // motor
			dJointSetHinge2Param (joint[0],dParamVel2,-speedA);
	    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[1],dParamVel2,-speedB);
	    dJointSetHinge2Param (joint[1],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[2],dParamVel2,-speedA);
	    dJointSetHinge2Param (joint[2],dParamFMax2,0.1);

      dJointSetHinge2Param (joint[3],dParamVel2,-speedB);
	    dJointSetHinge2Param (joint[3],dParamFMax2,0.1);

      dSpaceCollide (space,0,&nearCallback);
      dWorldQuickStep (world,TIME/TIME_DENOM);

      /*dsSetColor (1,1,0);
      dsSetTexture (DS_WOOD);
      dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
      dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
      dsSetColor (1,0,0);
      for (i=1; i<=2; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
      dsSetColor (1,1,1);
      for (i=3; i<=4; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);*/

    }


    // remove all contact joints
    dJointGroupEmpty (contactgroup);

    //
    // dVector3 ss;
    // dGeomBoxGetLengths (ground_box,ss);
    // dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);


  }
  /*dsSetColor (1,1,0);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,0,0);
  for (i=1; i<=2; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),WLENGTH,RADIUS);
  dsSetColor (1,1,1);
  for (i=3; i<=4; i++) dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),WLENGTH,RADIUS);*/

  dBodyCopyPosition(body[0],dpos);
  // Rotation matrix to Euler angle
  dBodyCopyRotation(body[0],drot);
  dyaw = atan2(drot[4],drot[0]);
  std::cout << "OUT : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << std::endl;
  NUM_SAMPLES++;
  // Quarternion to Euler angle
  // dBodyCopyQuaternion(body[0], dquar);
  // dQtoR(dquar,drot);
  // dyaw = atan2(drot[4],drot[0]);
  // std::cout << "Q.Get pose : " << dpos[0] << ", " << dpos[1] << ", " << dyaw/M_PI*180 << ", Time : " << time(NULL) << std::endl;
  return;

}

void init_ODE(){
  int i;
  dReal YAW = 0.0;
  // setup pointers to drawstuff callback functions
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

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
    sphere[i-1] = dCreateSphere(0,RADIUS);
    joint[i-1] = dJointCreateHinge2(world,0);
  }
  vehicle_def(0,0,YAW);

  // create car space and add it to the top level space
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
}


int main (int argc, char **argv)
{
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

  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dGeomDestroy (sphere[2]);
  dGeomDestroy (sphere[3]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
