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

#define _CRT_SECURE_NO_WARNINGS    // strcat 보안 경고로 인한 컴파일 에러 방지

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/scope_exit.hpp>
#include <boost/date_time.hpp>
#include <cstdlib>

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
#include <eigen_conversions/eigen_msg.h>

// DynaMoP
#include <DynaMoP/StateSpaces.h>
#include <DynaMoP/Scene.h>
#include <DynaMoP/DynaMoP.h>

#define NUM_BASE_DOF 3
#define MAX_COLUMN 1000
#define MAX_ROW 6
#define MAX_WORDS 20

#define SEARCH_AREA 1.0

// Global variables
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
std::vector<moveit_msgs::CollisionObject> tmp_collision_objects;

bool required_plan = true;

//Propagtion for kinodynamic
void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result){
    const auto *se2state = start->as<ompl::base::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    result->as<ompl::base::SE2StateSpace::StateType>()->setXY(//f(x,u,t) = u인 경우에 대한 propagtion
        pos[0] + ctrl[0] * duration * cos(rot),
        pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
        rot    + ctrl[1] * duration);
}
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

  std::cout<< "size_of_planning_scene : " << size_of_planning_scene <<std::endl;
  std::cout<< "Hololololololo~~~~" <<std::endl;

  if(size_of_planning_scene != 0){
    for(int i=0; i<size_of_planning_scene; i++)
      tmp_collision_objects.push_back(_msg->world.collision_objects[i]);
  }
  else
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamo_planner");
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    // ros::NodeHandle node_handle("~");
    ros::NodeHandle node_handle;

    double st[4] = {0.0, 0.0, 0.0, 1.0};
    double gl[4] = {-10.0, -10.0, 0.0, -M_PI/6};
    // Initialize the global variables
    start_pose.position.x = st[0];
    start_pose.position.y = st[1];
    start_pose.position.z = st[2];
    start_pose.orientation.w = st[3];
    // Goal pose is set as arbitrary values
    goal_pose.position.x = gl[0];
    goal_pose.position.y = gl[1];
    goal_pose.position.z = gl[2];
    goal_pose.orientation.w = gl[3];

    //Publisher
    ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/dynamop/move_group/display_planned_path", 1, true);
    ros::Publisher point_pub = node_handle.advertise<visualization_msgs::Marker>("/dynamop/StartGoalPoints",1);
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/dynamop/path_vis",1);
    // ros::Publisher vel_pub = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",100);
    //Subscriber
    ros::Subscriber start_pose_subscriber = node_handle.subscribe("/tf", 1, get_start_pose);
    ros::Subscriber goal_pose_subscriber = node_handle.subscribe("/move_base_simple/goal", 1, set_goal_pose);
    // ros::Subscriber goal_pose_subscriber = node_handle.subscribe("/goal", 1, set_goal_pose);
    ros::Subscriber scene_subscriber = node_handle.subscribe("/planning_scene", 1, get_planning_scene);

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
    scene->addCollisionObjects(tmp_collision_objects);
    // scene->addCollisionObjects();
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
    collisionCheckGroup = PLANNING_GROUP;

    ROS_INFO("num_dof = %d", num_dof);

    // State space, SE(2)
    auto state_space(std::make_shared<ompl::base::SE2StateSpace>());
    auto state_space3D(std::make_shared<ompl::base::SE3StateSpace>());
    // Control space
    auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 2));
    auto control_space3D(std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 3));

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
          points.color.g = 1.0f;
          points.color.b = 0.0f;
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


      // State space bounds
      ompl::base::RealVectorBounds bounds(2);
      // x
      bounds.setLow(0, std::min(start_pose.position.x-SEARCH_AREA, goal_pose.position.x-SEARCH_AREA));
      bounds.setHigh(0, std::max(start_pose.position.x+SEARCH_AREA, goal_pose.position.x+SEARCH_AREA));
      // y
      bounds.setLow(1, std::min(start_pose.position.y-SEARCH_AREA, goal_pose.position.y-SEARCH_AREA));
      bounds.setHigh(1, std::max(start_pose.position.y+SEARCH_AREA, goal_pose.position.y+SEARCH_AREA));
      state_space->setBounds(bounds);
      // State space bounds
      ompl::base::RealVectorBounds bounds3D(3);
      // x
      bounds3D.setLow(0, std::min(start_pose.position.x-SEARCH_AREA, goal_pose.position.x-SEARCH_AREA));
      bounds3D.setHigh(0, std::max(start_pose.position.x+SEARCH_AREA, goal_pose.position.x+SEARCH_AREA));
      // y
      bounds3D.setLow(1, std::min(start_pose.position.y-SEARCH_AREA, goal_pose.position.y-SEARCH_AREA));
      bounds3D.setHigh(1, std::max(start_pose.position.y+SEARCH_AREA, goal_pose.position.y+SEARCH_AREA));
      // z
      bounds3D.setLow(2, 0.13);
      bounds3D.setHigh(2, 0.13);
      state_space3D->setBounds(bounds3D);

      // Control space bounds
      ompl::base::RealVectorBounds control_bounds(2);
      control_bounds.setLow(-0.3);
      control_bounds.setHigh(0.3);
      control_space->setBounds(control_bounds);
      // Control space bounds
      ompl::base::RealVectorBounds control_bounds3D(3);
      control_bounds3D.setLow(-0.3);
      control_bounds3D.setHigh(0.3);
      control_space3D->setBounds(control_bounds3D);

      // Simpl setup
      ompl::control::SimpleSetupPtr simple_setup;
      ompl::control::SimpleSetupPtr simple_setup3D;
      simple_setup.reset(new ompl::control::SimpleSetup(control_space));
      simple_setup3D.reset(new ompl::control::SimpleSetup(control_space3D));
      // State propagation routine
      simple_setup->setStatePropagator(propagate);
      // State validity checking
      StateValidityCheckerPtr validity_checker;
      validity_checker.reset(new StateValidityChecker(simple_setup3D->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
      simple_setup->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

      typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> ScopedState;
      // Start state
      ScopedState Start(state_space);
      Start->setX(start_pose.position.x);// x
      Start->setY(start_pose.position.y);// y
      Start->setYaw(start_pose.orientation.w);// yaw
      // (*Start)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_pose.position.x;
      // (*Start)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_pose.position.y;
      // (*Start)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_pose.position.z;
      // (*Start)[1]->as<ompl::base::SO2StateSpace::StateType>()->setIdentity();
      // (*Start)[1]->as<ompl::base::SO2StateSpace::StateType>()->value            = start_pose.orientation.w;
      // Start->setX(st[0]);// x
      // Start->setY(st[1]);// y
      // Start->setYaw(st[2]);// yaw

      // Goal state
      // ; use the hard way to set the elements
      ScopedState Goal(state_space);
      Goal->setX(goal_pose.position.x);// x
      Goal->setY(goal_pose.position.y);// y
      Goal->setYaw(goal_pose.orientation.w);// yaw
      // (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_pose.position.x;
      // (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal_pose.position.y;
      // (*Goal)[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = goal_pose.position.z;
      // (*Goal)[1]->as<ompl::base::SO2StateSpace::StateType>()->setIdentity();
      // (*Goal)[1]->as<ompl::base::SO2StateSpace::StateType>()->value            = goal_pose.orientation.w;
      // Goal->setX(gl[0]);// x
      // Goal->setY(gl[1]);// y
      // Goal->setYaw(gl[2]);// yaw

      // Start and goal states
      simple_setup->setStartAndGoalStates(Start, Goal, 0.1);// setStartAndGoalStates(start, goal, threshold)

      ROS_INFO("Start : %f , %f , %f   Goal : %f , %f , %f", Start[0], Start[1], Start[2], Goal[0], Goal[1], Goal[2]);

      // Planning
      simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::control::DynaMoP(simple_setup->getSpaceInformation())));
      simple_setup->solve(ompl::base::timedPlannerTerminationCondition(10.0));

      if (simple_setup->haveSolutionPath()){
        std::cout << "Found solution:" << std::endl;

        // Store path
        ompl::control::PathControl &path = simple_setup->getSolutionPath();
//        path.printAsMatrix(std::cout);

        std::fstream fileout("/home/mrjohd/husky_ws/src/dynamo_planner/path.txt", std::ios::out);
        path.printAsMatrix(fileout);
        fileout.close();

        std::fstream filein("/home/mrjohd/husky_ws/src/dynamo_planner/path.txt", std::ios::in);

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
        uint NoPathPoints = path.getStateCount();//State
        //uint NoPathPoints = path.getControlCount();//Control
        robot_traj.joint_trajectory.joint_names = active_joint_names;
        robot_traj.joint_trajectory.points.resize(NoPathPoints);

        ROS_INFO("\nNo. of States = %d\n", NoPathPoints);
        //ROS_INFO("\nNo. of Controls = %d\n", NoPathPoints);

        //States
        typedef ompl::base::RealVectorStateSpace::StateType* StateTypePtr;
        typedef ompl::control::RealVectorControlSpace::ControlType* ControlTypePtr;

        for(uint i = 0; i < NoPathPoints; i++){
            //StateTypePtr rstate = static_cast<StateTypePtr>(path.getState(i));
            robot_traj.joint_trajectory.points[i].positions.resize(NUM_BASE_DOF);
            for (uint j = 0; j < NUM_BASE_DOF; j++){
                double traj_value = std::stod(static_cast<const std::string>(data[i][j]));
                //robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
                robot_traj.joint_trajectory.points[i].positions[j] = traj_value;

                vel.linear.x = std::stod(static_cast<const std::string>(data[i][3]));
                vel.linear.y = std::stod(static_cast<const std::string>(data[i][4]));
                vel.angular.z = std::stod(static_cast<const std::string>(data[i][5]));
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

        // vel_pub.publish(vel);

        // ros::Duration(1.0).sleep();
      }

      else
        std::cout << "No solution found" << std::endl;

      ros::spinOnce();
    }

    return 0;
}
