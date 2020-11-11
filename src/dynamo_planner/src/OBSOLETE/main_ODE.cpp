/***************************************************************************************
| * Author: Jinhyeok Jang                                                              |
| * Based on OMPL SST                                                                  |
| * Date : 2019 11 15 Fri                                                              |
| * SGVR lab : https://sgvr.kaist.ac.kr/                                               |
| * E-mail : antion001@gmail.com                                                       |
****************************************************************************************/
/***************************************************************************************
| * Author: Jinhyeok Jang                                                              |
| * Based on OMPL SST                                                                  |
| * Date : 2019 11 15 Fri                                                              |
| * SGVR lab : https://sgvr.kaist.ac.kr/                                               |
| * E-mail : antion001@gmail.com                                                       |
****************************************************************************************/

#define _CRT_SECURE_NO_WARNINGS    // strcat 보안 경고로 인한 컴파일 에러 방지.

#include <ros/ros.h>

// c++
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cstdlib>

// Messages
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "dynamo_planner/custom_states_msgs.h"
#include <sensor_msgs/JointState.h>

// Boost
#include <boost/scope_exit.hpp>
#include <boost/date_time.hpp>

// ompl
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include "ompl/control/PathControl.h"
#include "ompl/geometric/PathGeometric.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/config.h>

//Open Dynamics Engine(ODE)
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ode/ode.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen_conversions/eigen_msg.h>

// DynaMoP
#include <DynaMoP/StateSpaces.h>
#include <DynaMoP/Scene.h>
#include <DynaMoP/DynaMoP.h>

#define NUM_BASE_DOF 3
#define MAX_COLUMN 1000
#define MAX_ROW 6
#define MAX_WORDS 20

#define PLANNING_TIME   30
#define SEARCH_AREA 30.0
#define VEL_BOUND 10.0
#define CUR_BOUND 0.9
#define TIME_DENOM  1.0

#define MIN_DIST 0.5

// two-wheeled model
#define DIM_STATE 5 //x y yaw v omega
#define DIM_CTRL 2  //wL wR

typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> ScopedState;

typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
typedef ompl::control::RealVectorControlSpace::ControlType* ControlTypePtr;

/*** Global variables ***/
// Start & goal pose
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
double st[3] = {0.0, 0.0, -M_PI};
double gl[3] = {20.0, 0.0, -M_PI};
// double gl[3] = {5.490, -5.415, -M_PI/6};
// double gl[3] = {-17.490, 10.415, -M_PI/6};
// Gazebo map objects
std::vector<moveit_msgs::CollisionObject> tmp_collision_objects;
// New Goal set flag
bool required_plan = true;
// Map loading
bool map_loading = false;
// Sample
visualization_msgs::Marker samples;
geometry_msgs::Point sample_pt;
visualization_msgs::Marker props;
geometry_msgs::Point prop_pt;
int call_cnt=0;

class RigidBodyEnvironment : public ompl::control::OpenDEEnvironment{
public:
    RigidBodyEnvironment(){
        createWorld();
    }
    ~RigidBodyEnvironment() override{
        destroyWorld();
    }

    /**************************************************
     * Implementation of functions needed by planning *
     **************************************************/
    unsigned int getControlDimension() const override{
        return 3;
    }

    void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const override{
        static double maxForce = 0.2;
        lower.resize(3);
        lower[0] = -maxForce;
        lower[1] = -maxForce;
        lower[2] = -maxForce;

        upper.resize(3);
        upper[0] = maxForce;
        upper[1] = maxForce;
        upper[2] = maxForce;
    }

    void applyControl(const double *control) const override{
        dBodyAddForce(boxBody, control[0], control[1], control[2]);
    }

    bool isValidCollision(dGeomID /*geom1*/, dGeomID /*geom2*/, const dContact & /*contact*/) const override{
        return false;
    }

    void setupContact(dGeomID /*geom1*/, dGeomID /*geom2*/, dContact &contact) const override{
        contact.surface.mode = dContactSoftCFM | dContactApprox1;
        contact.surface.mu = 0.9;
        contact.surface.soft_cfm = 0.2;
    }

    /**************************************************/

    // OMPL does not require this function here; we implement it here
    // for convenience. This function is only OpenDE code to create a
    // simulation environment. At the end of the function, there is a
    // call to setPlanningParameters(), which configures members of
    // the base class needed by planners.
    void createWorld();

    // Clear all OpenDE objects
    void destroyWorld();

    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters();

    // the simulation world
    dWorldID bodyWorld;

    // the space for all objects
    dSpaceID space;

    // the car mass
    dMass m;

    // the body geom
    dGeomID boxGeom;

    // the body
    dBodyID boxBody;
};

// Define the goal we want to reach
class RigidBodyGoal : public ompl::base::GoalRegion{
public:
    RigidBodyGoal(const ompl::base::SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
    {
        threshold_ = 0.5;
    }

    double distanceGoal(const ompl::base::State *st) const override
    {
        const double *pos = st->as<ompl::control::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(pos[0] - 30);
        double dy = fabs(pos[1] - 55);
        double dz = fabs(pos[2] - 35);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
};

// Define how we project a state
class RigidBodyStateProjectionEvaluator : public ompl::base::ProjectionEvaluator{
public:
    RigidBodyStateProjectionEvaluator(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 3;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(3);
        cellSizes_[0] = 1;
        cellSizes_[1] = 1;
        cellSizes_[2] = 1;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const double *pos = state->as<ompl::control::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        projection[0] = pos[0];
        projection[1] = pos[1];
        projection[2] = pos[2];
    }
};

// Define our own space, to include a distance function we want and register a default projection
class RigidBodyStateSpace : public ompl::control::OpenDEStateSpace{
public:
    RigidBodyStateSpace(const ompl::control::OpenDEEnvironmentPtr &env) : ompl::control::OpenDEStateSpace(env){
    }

    double distance(const ompl::base::State *s1, const ompl::base::State *s2) const override{
        const double *p1 = s1->as<ompl::control::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        const double *p2 = s2->as<ompl::control::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(p1[0] - p2[0]);
        double dy = fabs(p1[1] - p2[1]);
        double dz = fabs(p1[2] - p2[2]);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    void registerProjections() override{
        registerDefaultProjection(std::make_shared<RigidBodyStateProjectionEvaluator>(this));
    }
};

/***********************************************
 * Member function implementations             *
 ***********************************************/

void RigidBodyEnvironment::createWorld(){
    // BEGIN SETTING UP AN OPENDE ENVIRONMENT
    // ***********************************
    bodyWorld = dWorldCreate();
    space = dHashSpaceCreate(nullptr);

    dWorldSetGravity(bodyWorld, 0, 0, -0.981);

    double lx = 0.2;
    double ly = 0.2;
    double lz = 0.1;

    dMassSetBox(&m, 1, lx, ly, lz);
    dMassAdjust(&m, 10.);

    boxGeom = dCreateBox(space, lx, ly, lz);
    boxBody = dBodyCreate(bodyWorld);
    dBodySetMass(boxBody, &m);
    dGeomSetBody(boxGeom, boxBody);

    // *********************************
    // END SETTING UP AN OPENDE ENVIRONMENT

    setPlanningParameters();
}

void RigidBodyEnvironment::destroyWorld(){
    dSpaceDestroy(space);
    dWorldDestroy(bodyWorld);
}

void RigidBodyEnvironment::setPlanningParameters(){
    // Fill in parameters for OMPL:
    world_ = bodyWorld;
    collisionSpaces_.push_back(space);
    stateBodies_.push_back(boxBody);
    stepSize_ = 0.05;
    maxContacts_ = 3;
    minControlSteps_ = 10;
    maxControlSteps_ = 500;
}

class PlannerAndGazeboTest{
public:
  PlannerAndGazeboTest(){
    //Publish
    pub_ = n_.advertise<dynamo_planner::custom_states_msgs>("/PlannerStates", 1);//13kHz
    sample_pub_ = n_.advertise<visualization_msgs::Marker>("/Samples",1);
    prop_pub_ = n_.advertise<visualization_msgs::Marker>("/Props",1);
    //Subscribe
    sub_  = n_.subscribe("/GazeboStates", 1, &PlannerAndGazeboTest::GazeboToPlanner, this);//5.1kHz
    sub_js= n_.subscribe("/joint_states", 1, &PlannerAndGazeboTest::JointStatesCallBack, this);

    samples.header.frame_id ="world";
    samples.header.stamp= ros::Time();
    samples.ns="Samples";
    samples.id = 3;
    samples.type = visualization_msgs::Marker::POINTS;
    samples.scale.x = 0.1;
    samples.scale.y = 0.1;
    samples.color.a = 1.0;
    samples.color.r = 0.8f;
    samples.color.b = 0.2f;
    samples.color.g = 0.8f;

    props.header.frame_id ="world";
    props.header.stamp= ros::Time();
    props.ns="PropStates";
    props.id = 2;
    props.type = visualization_msgs::Marker::POINTS;
    props.scale.x = 0.1;
    props.scale.y = 0.1;
    props.color.a = 1.0;
    props.color.r = 0.8f;
    props.color.b = 0.8f;
    props.color.g = 0.1f;

   std::cout<< "I'm alive!" << std::endl;
  }
  ~PlannerAndGazeboTest(){
    this->n_.shutdown();
    free(sub_);
    free(pub_);
    Next_state.flag = false;
    std::cout<< "I'm dead" << std::endl;
  }
  //Call-back function
  void GazeboToPlanner(const dynamo_planner::custom_states_msgs& _msg){
    Next_state.pre_x = _msg.pre_x;
    Next_state.pre_y = _msg.pre_y;
    Next_state.pre_yaw = _msg.pre_yaw;
    Next_state.flag = _msg.flag;
    // std::cout << Next_state << std::endl;
  }
  void JointStatesCallBack(const sensor_msgs::JointState::ConstPtr& _msg){
    // std::cout << "tf_cb" << std::endl;
    Next_state.x = _msg->position[0];
    Next_state.y = _msg->position[1];
    Next_state.yaw = _msg->position[2];
  }
  //Propagation mode selection
  static void calculate(){
      dynamo_planner::custom_states_msgs differ_meter;
      do{
        pub_.publish(Curr_States);//Enable gazebo plugin's call back
        ros::spinOnce();//Subscribe gazebo plugin's propagated state

        differ_meter.diff_x = Curr_States.x == Next_state.pre_x ? true:false;
        differ_meter.diff_y = Curr_States.y == Next_state.pre_y ? true:false;
        differ_meter.diff_yaw = Curr_States.yaw == Next_state.pre_yaw ? true:false;
        // std::cout << int(Next_state.flag) << std::endl;
      }
      while(!differ_meter.diff_x || !differ_meter.diff_y || !differ_meter.diff_yaw);
      // while(!Next_state.flag);
      Next_state.flag = false;

      // Next_state.x = Curr_States.x + Curr_States.controlX * Curr_States.duration;
      // Next_state.y = Curr_States.y + Curr_States.controlY * Curr_States.duration;
      // Next_state.yaw = Curr_States.yaw    + Curr_States.controlYAW * Curr_States.duration;
  }
  //Faster propagator
  static void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result){
      const auto *se2state = start->as<ompl::base::SE2StateSpace::StateType>();
      const double* pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
      const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
      const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
      //Show samples
      sample_pt.x = pos[0];
      sample_pt.y = pos[1];
      samples.points.push_back(sample_pt);
      sample_pub_.publish(samples);// Publish samples
      // std::cout << "samples[0] : " << samples.points[0] << std::endl;

      //Publish current sample
      // dynamo_planner::custom_states_msgs Curr_States;
      Curr_States.x = pos[0];
      Curr_States.y = pos[1];
      Curr_States.yaw = rot;

      Curr_States.controlX = ctrl[0]*cos(rot);
      Curr_States.controlY = ctrl[0]*sin(rot);
      Curr_States.controlYAW = ctrl[0]*ctrl[1];

      Curr_States.duration = duration/TIME_DENOM;

      std::cout << "\nCurr_States : " << Curr_States.x << ", " << Curr_States.y << ", " << Curr_States.yaw;
      std::cout << " Controls : " << ctrl[0] << ", " << ctrl[1] << std::endl;

      // ros::Time begin = ros::Time::now();
      calculate();

      result->as<ompl::base::SE2StateSpace::StateType>()->setXY(Next_state.x, Next_state.y);
      result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(Next_state.yaw);

      //Show prop states
      prop_pt.x = Next_state.x;
      prop_pt.y = Next_state.y;
      props.points.push_back(prop_pt);
      prop_pub_.publish(props);// Publish samples

  }

  //Parameters
  double* position;
  double rotation;
  double* control;
  double duration;

private:
  //Ros setting
  ros::NodeHandle n_;
  // ros::Publisher pub_;
  static ros::Publisher pub_;
  static ros::Publisher sample_pub_;
  static ros::Publisher prop_pub_;
  ros::Subscriber sub_;
  // static ros::Subscriber sub_;
  ros::Subscriber sub2_;
  ros::Subscriber sub_js;
  // Planner-Gazebo states
  static dynamo_planner::custom_states_msgs Curr_States;
  static dynamo_planner::custom_states_msgs Next_state;


};//End of class PlannerAndGazeboTest
ros::Publisher PlannerAndGazeboTest::pub_;
// ros::Subscriber PlannerAndGazeboTest::sub_;
ros::Publisher PlannerAndGazeboTest::sample_pub_;
ros::Publisher PlannerAndGazeboTest::prop_pub_;
dynamo_planner::custom_states_msgs PlannerAndGazeboTest::Curr_States;
dynamo_planner::custom_states_msgs PlannerAndGazeboTest::Next_state;

//Get start position from world
void get_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg){
    // Convert tf2_msgs to geometry_msgs::Pose
    // std::cout << " _msg->transforms.size() : " <<  _msg->transforms.size() << std::endl;
    for(int i = 0; i < _msg->transforms.size(); i++){
        if(std::string("world").compare(_msg->transforms.at(i).child_frame_id) == 0){
            start_pose.position.x = _msg->transforms.at(i).transform.translation.x;
            start_pose.position.y = _msg->transforms.at(i).transform.translation.y;
            start_pose.position.z = 0.0;

            start_pose.orientation.w = _msg->transforms.at(i).transform.rotation.w;
            start_pose.orientation.x = _msg->transforms.at(i).transform.rotation.x;
            start_pose.orientation.y = _msg->transforms.at(i).transform.rotation.y;
            start_pose.orientation.z = _msg->transforms.at(i).transform.rotation.z;
            ROS_INFO("Get start!");
            break;
        }
    }

//    std::cout << "Set start pose" << std::endl;
}
//Set goal point on world
void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    goal_pose = _msg->pose;
    required_plan = true;
    ROS_INFO("Set goal!");
}
//Get moveit! planning scene
void get_planning_scene(const moveit_msgs::PlanningScene::ConstPtr& _msg){
  std::vector<moveit_msgs::CollisionObject> tmp(std::begin(_msg->world.collision_objects), std::end(_msg->world.collision_objects));
  double size_of_planning_scene = tmp.size();
  tmp_collision_objects.clear();
  ROS_INFO("Size of the planning_scene : %f", size_of_planning_scene);
  if(size_of_planning_scene != 0){
    for(int i=0; i<size_of_planning_scene; i++){
      if(_msg->world.collision_objects[i].id.find("plane") == std::string::npos){
        //해당 obstacle id에 plane이 없는 경우 맵에 추가
        ROS_INFO("Obstacles : %s", _msg->world.collision_objects[i].id.c_str());
        tmp_collision_objects.push_back(_msg->world.collision_objects[i]);
      }
      else
        continue;
    }
    map_loading = true;
    return;
  }
  else{
    ROS_INFO("Nothing...");
    map_loading = true;
    return;
  }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamo_planner");
    // initialize OpenDE
    dInitODE2(0);
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    ros::NodeHandle node_handle("~");
    // ros::NodeHandle node_handle;
    // Initialize the global variables
    start_pose.position.x = st[0];
    start_pose.position.y = st[1];
    start_pose.orientation.w = st[2];
    // Goal pose is set as arbitrary values
    goal_pose.position.x = gl[0];
    goal_pose.position.y = gl[1];
    goal_pose.orientation.w = gl[2];

    //Publisher
    // ros::Publisher state_pub = node_handle.advertise<dynamo_planner::custom_states_msgs>("/PlannerStates",1);
    ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher point_pub = node_handle.advertise<visualization_msgs::Marker>("/StartGoalPoints",1);
    ros::Publisher sample_pub = node_handle.advertise<visualization_msgs::Marker>("/Samples_last",1);
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path_vis",1);
    // ros::Publisher vel_pub = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",100);
    //Subscriber
    // ros::Subscriber start_pose_subscriber = node_handle.subscribe("/tf", 1, get_start_pose);
    ros::Subscriber goal_pose_subscriber = node_handle.subscribe("/move_base_simple/goal", 1, set_goal_pose);// ros::Subscriber goal_pose_subscriber = node_handle.subscribe("/goal", 1, set_goal_pose);
    ros::Subscriber scene_subscriber = node_handle.subscribe("/gazebo_planning_scene", 1, get_planning_scene);
    // ros::Subscriber state_subscribe = node_handle.subscribe("/PlannerStates", 1, get_planner_states);
    //Parameters
    double param;
    node_handle.getParam("param",param);
    ROS_INFO("Got parameter : %f sec planning time", param);
    // Please set your group in moveit!.
    const std::string PLANNING_GROUP = "vehicle";
    const std::string BASE_GROUP = "base_body";
    // const std::string MANI_COLL_CHECK_GROUP = "without_right_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(BASE_GROUP);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));//moveit_core::planning_scene::PlanningScenePtr

    // moveit planning scene
    ScenePtr scene(new Scene(planning_scene, "world"));

    //// Please set your start configuration and predefined configuration.
    std::vector<double> s_conf, pre_conf;
    joint_model_group->getVariableDefaultPositions(s_conf);

    joint_model_group->getVariableDefaultPositions(pre_conf);
    pre_conf.erase(pre_conf.begin(), pre_conf.begin()+NUM_BASE_DOF);

    // state space
    std::string collisionCheckGroup;
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    collisionCheckGroup = BASE_GROUP;

    ROS_INFO("num_dof = %d", num_dof);

    // create the OpenDE environment
    ompl::control::OpenDEEnvironmentPtr env(std::make_shared<RigidBodyEnvironment>());
    // create the state space and the control space for planning
    auto stateSpace = std::make_shared<RigidBodyStateSpace>(env);
    // this will take care of setting a proper collision checker and the starting state for the planner as the initial
    // OpenDE state


    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-SEARCH_AREA);
    bounds.setHigh(SEARCH_AREA);
    stateSpace->setVolumeBounds(bounds);

    bounds.setLow(-VEL_BOUND);
    bounds.setHigh(VEL_BOUND);
    stateSpace->setLinearVelocityBounds(bounds);
    stateSpace->setAngularVelocityBounds(bounds);

    ros::Rate rate(1);//1Hz

    while(node_handle.ok()){
      // wait until the goal is set
      if(!required_plan){
          ros::spinOnce();
          scene->addCollisionObjects(tmp_collision_objects);
          scene->updateCollisionScene();

          // Start, Goal marker
          visualization_msgs::Marker points;
          geometry_msgs::Point pt;
          points.header.frame_id ="world";
          points.header.stamp= ros::Time();

          points.ns="Start_Pt";
          points.id = 0;
          points.type = visualization_msgs::Marker::POINTS;
          points.scale.x = 0.3;
          points.scale.y = 0.3;
          points.color.a = 1.0; // Don't forget to set the alpha!
          points.color.r = 0.0f;
          points.color.g = 0.0f;
          points.color.b = 1.0f;
          pt.x = start_pose.position.x;
          pt.y = start_pose.position.y;
          points.points.push_back(pt);
          point_pub.publish(points);//Pulish Start

          points.points.clear();
          points.ns="Goal_Pt";
          points.id = 1;
          points.type = visualization_msgs::Marker::POINTS;
          points.scale.x = 0.3;
          points.scale.y = 0.3;
          points.color.r = 1.0f;
          points.color.b = 0.0f;
          points.color.g = 0.0f;
          pt.x = goal_pose.position.x;
          pt.y = goal_pose.position.y;
          points.points.push_back(pt);
          point_pub.publish(points);// Publish Goal
          continue;
      }
      required_plan = false;
      samples.points.clear();
      props.points.clear();

      ROS_INFO("Initializing planning sequence!");
      while(!map_loading)  ros::spinOnce();
      ROS_INFO("Load planning scene!");
      scene->addCollisionObjects(tmp_collision_objects);
      scene->updateCollisionScene();

      // Simpl setup
      ompl::control::OpenDESimpleSetup ODESimpleSetup(stateSpace);
      // State validity checking
      StateValidityCheckerPtr validity_checker;
      validity_checker.reset(new StateValidityChecker(ODESimpleSetup.getSpaceInformation(), planning_scene, BASE_GROUP, collisionCheckGroup));
      ODESimpleSetup.setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

      // Start state
      ScopedState Start(stateSpace);
      Start->setX(st[0]);// x
      Start->setY(st[1]);// y
      Start->setYaw(st[2]);// yaw

      // Goal state
      ScopedState Goal(stateSpace);
      double quatx,quaty,quatz,quatw;
      double roll,pitch,yaw;
      quatx = goal_pose.orientation.x;
      quaty = goal_pose.orientation.y;
      quatz = goal_pose.orientation.z;
      quatw = goal_pose.orientation.w;
      tf2::Quaternion quat(quatx,quaty,quatz,quatw);
      tf2::Matrix3x3 mat(quat);
      mat.getRPY(roll,pitch,yaw);
      Goal->setX(goal_pose.position.x);// x
      Goal->setY(goal_pose.position.y);// y
      Goal->setYaw(yaw);// yaw

      // Start and goal states
      ODESimpleSetup.setStartAndGoalStates(Start, Goal, 0.1);// setStartAndGoalStates(start, goal, threshold)
      ODESimpleSetup.setup();
      ODESimpleSetup.print();

      ROS_INFO("Start : %f , %f , %f   \nGoal : %f , %f , %f", Start[0], Start[1], Start[2], Goal[0], Goal[1], Goal[2]);

      // Planning
      ODESimpleSetup.setPlanner(ompl::base::PlannerPtr(new ompl::control::DynaMoP(ODESimpleSetup.getSpaceInformation())));
      ODESimpleSetup.solve(ompl::base::timedPlannerTerminationCondition(param));

      if (ODESimpleSetup.haveSolutionPath()){
        ROS_INFO("Solution Found!!!");
        ROS_INFO("Number of samples : %lu", samples.points.size());

        // Store path
        ompl::control::PathControl &path = ODESimpleSetup.getSolutionPath();
        // path.printAsMatrix(std::cout);

        std::fstream fileout("/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/path/path.txt", std::ios::out);
        path.printAsMatrix(fileout);
        fileout.close();

        std::fstream filein("/home/mrjohd/Kinodynamic_ws/src/dynamo_planner/path/path.txt", std::ios::in);

        char word;
        char data[MAX_COLUMN][MAX_ROW][MAX_WORDS]={0};
        int i=0, j=0, k;
        //Read text
        while(filein.get(word)){
           // filein.get(word);
            // std::cout << "Word : " << word << std::endl;
            if((word == ' ') || (word == '\n') || (k>=MAX_WORDS)){
              //Next column
              k=0;
              j++;
              if(j>=MAX_ROW){
              //Next row
                  j=0;
                  i++;
              }
            }
            else{
              data[i][j][k] = word;
              k++;
            }
        }
        filein.close();

        //Robot trajectory
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::RobotTrajectory robot_traj;

        //Velocity publish
        geometry_msgs::Twist vel;

        //Path visualize
        nav_msgs::Path path_vis;
        geometry_msgs::PoseStamped robot_pose;

        path_vis.poses.clear();
        path_vis.header.stamp = ros::Time::now();
        path_vis.header.frame_id = "world";

        const moveit::core::JointModelGroup* model_group = planning_scene->getRobotModel()->getJointModelGroup(BASE_GROUP);
        const std::vector<std::string>& active_joint_names = model_group->getActiveJointModelNames();
        int NoPathPoints = path.getStateCount();//State
        //uint NoPathPoints = path.getControlCount();//Control
        robot_traj.joint_trajectory.joint_names = active_joint_names;
        robot_traj.joint_trajectory.points.resize(NoPathPoints);

        ROS_INFO("No. of States = %d", NoPathPoints);
        //ROS_INFO("\nNo. of Controls = %d\n", NoPathPoints);

        //States
        for(uint i = 0; i < NoPathPoints; i++){
            //StateTypePtr rstate = static_cast<StateTypePtr>(path.getState(i));
            robot_traj.joint_trajectory.points[i].positions.resize(NUM_BASE_DOF);
            for (uint j = 0; j < NUM_BASE_DOF; j++){
                double traj_value = std::stod(static_cast<const std::string>(data[i][j]));
                //robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
                robot_traj.joint_trajectory.points[i].positions[j] = traj_value;
            }
            robot_pose.pose.position.x = robot_traj.joint_trajectory.points[i].positions[0];
            robot_pose.pose.position.y = robot_traj.joint_trajectory.points[i].positions[1];
            path_vis.poses.push_back(robot_pose);

            robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        display_trajectory.trajectory.push_back(robot_traj);
        display_pub.publish(display_trajectory);

        //Path visulaization
        path_pub.publish(path_vis);
        dCloseODE();
      }

      else
        ROS_INFO("No solution found");
    }

    return 0;
}
