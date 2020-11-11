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
#include <dynamo_planner/custom_states_msgs.h>
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
// #include "../../../OMPL/src/ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/config.h>

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
#define SEARCH_AREA 15.0
#define VEL_BOUND 10
#define CUR_BOUND M_PI/4.
#define TIME_DENOM  1.0

#define MIN_DIST 0.5

typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> ScopedState;

typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
typedef ompl::control::RealVectorControlSpace::ControlType* ControlTypePtr;

/*** Global variables ***/
// Start & goal pose
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
double st[3] = {0.0, 0.0, 0.0};
// double gl[3] = {5.0, -5.0, 0.0};
double gl[3] = {8.0, -2.5, -M_PI};
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
double PropTime=0.;
class PlannerAndODE
{
public:
  PlannerAndODE(){
    //Publish
    pub_ = n_.advertise<dynamo_planner::custom_states_msgs>("/PlannerStates", 1);//13kHz
    sample_pub_ = n_.advertise<visualization_msgs::Marker>("/Samples",1);
    prop_pub_ = n_.advertise<visualization_msgs::Marker>("/Props",1);
    //Subscribe
    sub_  = n_.subscribe("/Prop_States", 1, &PlannerAndODE::PropagatorToPlanner, this);//20Hz
    // sub_js= n_.subscribe("/joint_states", 1, &PlannerAndODE::JointStatesCallBack, this);

    Curr_States.index = 0;
    Next_state.index = 0;
    Curr_States.flag = false;
    Next_state.flag = false;
    Next_state.sub_flag=true;
    Curr_States.sub_flag=true;

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
  ~PlannerAndODE(){
    this->n_.shutdown();
    free(sub_);
    free(pub_);
    Next_state.flag = false;
    std::cout<< "I'm dead" << std::endl;
  }
  //Call-back function
  void PropagatorToPlanner(const dynamo_planner::custom_states_msgs& _msg){
    Next_state.pre_x = _msg.pre_x;
    Next_state.pre_y = _msg.pre_y;
    Next_state.pre_yaw = _msg.pre_yaw;

    Next_state.x = _msg.x;
    Next_state.y = _msg.y;
    Next_state.yaw = _msg.yaw;
    // Next_state = _msg;

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
      // Synchronize published state and subscribed state pair
      bool differ_x,differ_y,differ_yaw,differ_c1,differ_c2;
      do{
        pub_.publish(Curr_States);//Enable gazebo plugin's call back
        ros::spinOnce();//Subscribe gazebo plugin's propagated state

        differ_x = (Curr_States.x == Next_state.pre_x) ? true:false;
        differ_y = (Curr_States.y == Next_state.pre_y) ? true:false;
        differ_yaw = (Curr_States.yaw == Next_state.pre_yaw) ? true:false;
        // differ_c1 = Curr_States.controlA == Next_state.controlA ? true:false;
        // differ_c2 = Curr_States.controlB == Next_state.controlB ? true:false;
        // std::cout << int(Next_state.flag) << std::endl;
      }
      while(!differ_x || !differ_y || !differ_yaw);

      // while(!Next_state.flag);

      /*
      dynamo_planner::custom_states_msgs Temp_States;
      Temp_States.x = 5.0;
      Temp_States.y = 5.0;
      Temp_States.yaw = 0.0;
      Temp_States.flag = Next_state.flag;
      do{
        pub_.publish(Curr_States);//Enable gazebo plugin's call back
        Curr_States.flag = true;
        ros::spinOnce();//Subscribe gazebo plugin's propagated state
      }
      while(Next_state.flag != Curr_States.flag);
      */

      // std::cout << "\nCurr_States : " << Curr_States.x << ", " << Curr_States.y << ", " << Curr_States.yaw*180/M_PI;


      // pub_.publish(Curr_States);//Enable gazebo plugin's call back
      // ros::spinOnce();//Subscribe gazebo plugin's propagated state

      // Next_state.x = Curr_States.x + Curr_States.controlX * Curr_States.duration;
      // Next_state.y = Curr_States.y + Curr_States.controlY * Curr_States.duration;
      // Next_state.yaw = Curr_States.yaw    + Curr_States.controlYAW * Curr_States.duration;
/*
      Next_state.yaw = Curr_States.yaw    + (Curr_States.controlB-Curr_States.controlA) * Curr_States.duration;
      Next_state.x = Curr_States.x + (Curr_States.controlA+Curr_States.controlB) * cos(Next_state.yaw) * Curr_States.duration;
      Next_state.y = Curr_States.y + (Curr_States.controlA+Curr_States.controlB) * sin(Next_state.yaw) * Curr_States.duration;

      Next_state.flag = false;
*/
  }
  //Faster propagator
  static void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result)
  {
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

      // Curr_States.controlX = ctrl[0]*cos(rot);
      // Curr_States.controlY = ctrl[0]*sin(rot);
      Curr_States.controlA = ctrl[0];
      Curr_States.controlB = ctrl[1];

      Curr_States.duration = duration/TIME_DENOM;
      // std::cout << "\nCurr_States : " << Curr_States.x << ", " << Curr_States.y << ", " << Curr_States.yaw;
      // std::cout << " Controls : " << ctrl[0] << ", " << ctrl[1] << std::endl;

      ros::Time begin = ros::Time::now();
      calculate();
      ros::Time end = ros::Time::now();
      double propagation_time = end.toSec() - begin.toSec();
      PropTime += propagation_time;
      // std::cout << "PropTime : " << PropTime << std::endl;
      // std::cout << "propagation_time : " << propagation_time << std::endl;

      result->as<ompl::base::SE2StateSpace::StateType>()->setXY(Next_state.x, Next_state.y);
      result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(Next_state.yaw);
      //Show prop states
      prop_pt.x = Next_state.x;
      prop_pt.y = Next_state.y;
      props.points.push_back(prop_pt);
      prop_pub_.publish(props);// Publish samples

      Curr_States.flag = false;
      Next_state.flag = false;
      Curr_States.sub_flag=Next_state.sub_flag;

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

};//End of class PlannerAndODE
ros::Publisher PlannerAndODE::pub_;
// ros::Subscriber PlannerAndODE::sub_;
ros::Publisher PlannerAndODE::sample_pub_;
ros::Publisher PlannerAndODE::prop_pub_;
dynamo_planner::custom_states_msgs PlannerAndODE::Curr_States;
dynamo_planner::custom_states_msgs PlannerAndODE::Next_state;

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
    // obstacles from scene.cpp
    scene->addCollisionObjects();
    scene->updateCollisionScene();

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

    // State space, SE(2)
    auto state_space(std::make_shared<ompl::base::SE2StateSpace>());
    // Control space
    auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 2));

    // 2D
    // State space bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-SEARCH_AREA);
    bounds.setHigh(SEARCH_AREA);
    state_space->setBounds(bounds);

    // Control space bounds
    ompl::base::RealVectorBounds control_bounds(2);
    /*// Acceleration(thrust)
    control_bounds.setLow(0,-VEL_BOUND);
    control_bounds.setHigh(0,VEL_BOUND);
    // Curvature(husky) or steering(Ackerman model)(or steering)
    control_bounds.setLow(1,-CUR_BOUND);
    control_bounds.setHigh(1,CUR_BOUND);*/
    control_bounds.setLow(-VEL_BOUND);
    control_bounds.setHigh(VEL_BOUND);
    control_space->setBounds(control_bounds);

    // ros::Rate rate(1);//1Hz


    while(ros::ok()){
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

          PropTime = 0.0;

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
      ompl::control::SimpleSetupPtr simple_setup;
      simple_setup.reset(new ompl::control::SimpleSetup(control_space));

      // State propagation routine
      PlannerAndODE Planner_n_ODE;// PlannerAndGazebo Planner_n_Gazebo;
      simple_setup->setStatePropagator(Planner_n_ODE.propagate);
      // State validity checking
      StateValidityCheckerPtr validity_checker;
      validity_checker.reset(new StateValidityChecker(simple_setup->getSpaceInformation(), planning_scene, BASE_GROUP, collisionCheckGroup));
      simple_setup->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

      // Start state
      ScopedState Start(state_space);
      Start->setX(st[0]);// x
      Start->setY(st[1]);// y
      Start->setYaw(st[2]);// yaw

      // Goal state
      ScopedState Goal(state_space);
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
      Goal->setYaw(gl[2]);// yaw

      // Start and goal states
      simple_setup->setStartAndGoalStates(Start, Goal, 1.5);// setStartAndGoalStates(start, goal, threshold)

      ROS_INFO("Start : %f , %f , %f   \nGoal : %f , %f , %f", Start[0], Start[1], Start[2], Goal[0], Goal[1], Goal[2]);

      // Planning
      ros::Time plannerST = ros::Time::now();
      ROS_INFO("Set planner...");
      simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::control::DynaMoP(simple_setup->getSpaceInformation())));
      ROS_INFO("Planning...");
      ompl::base::ProblemDefinitionPtr pdef = simple_setup->getProblemDefinition();
      simple_setup->solve(plannerOrTerminationCondition (ompl::base::exactSolnPlannerTerminationCondition(pdef), ompl::base::timedPlannerTerminationCondition(param)));
      // simple_setup->solve(ompl::base::timedPlannerTerminationCondition(param));
      // simple_setup->solve(ompl::base::timedPlannerTerminationCondition(PLANNING_TIME));
      ros::Time plannerET = ros::Time::now();

      if (simple_setup->haveSolutionPath()){
        ROS_INFO("Solution Found!!!");
        ROS_INFO("Number of samples : %lu", samples.points.size());

        // Store path
        ompl::control::PathControl &path = simple_setup->getSolutionPath();
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
      }

      else
        ROS_INFO("No solution found");


      double planningTime = plannerET.toSec()-plannerST.toSec();
      ROS_INFO("Start Time              : %f sec",plannerST.toSec());
      ROS_INFO("End Time                : %f sec",plannerET.toSec());
      ROS_INFO("Total planning time     : %f sec",planningTime);
      ROS_INFO("Propagation time        : %f sec",PropTime);
      ROS_INFO("Propagation proportion  : %f %%", PropTime/planningTime*100);
    }

    // ros::Duration(1.0).sleep();
    return 0;
}
