# DynamoP 2.0

  Revised DynamoP

# Install dependancies

1.  Install ros
    Link for installing ros-melodic version  
    <http://wiki.ros.org/melodic/Installation/Ubuntu>  
    Follow the instruction.  

2.  Once install ros, you need to define a workspace.  
    <http://wiki.ros.org/ko/catkin/Tutorials/create_a_workspace>  
    Follow the indtruction. By the way, instead of 'catkin_ws', name it as 'MotionPLanning_ws'  

3.  Install OMPL - Opensource Motion Planning Library.  
    <http://ompl.kavrakilab.org/installation.html>  
    Follow the link, and go to 'ROS' tab and follow the instruction.  

4.  Install Moveit! - for collision checking  
    <https://moveit.ros.org/install/>  
    Follow the instruction.  

-   OMPL github...This would be helpful  
    <https://github.com/ompl/ompl/tree/master/src/ompl>  

# How to run

1.  Open the terminal and
    ```bash
    cd ~/MotionPlanning_ws
    ```
2.  After get to MotionPlanning_ws, enter
    ```bash
    source devel/setup.sh
    ```
      This will make your shell as a ROS workspace.  
      **_Should do this once you make new shell._**  
      To check if your workspace setting is correct...  
    ```bash
    echo $ROS_PACKAGE_PATH        
    ```
      If the setting is not correct you would see  
    ```bash
    $/opt/ros/$(ROS_DISTRO)/share
    ```
       _ex)_ /opt/ros/melodic/share  
       Unless you would see  
    ```bash
    $/home/mrjohd/MotionPlanning_ws/src:/opt/ros/$(ROS_DISTRO)/share
    ```
3.  If you are ...  
    **A. using CLion**  
    Enter following command (Should set the current shell as workspace!!)  

    ```bash
    sh ~/clion/bin/clion.sh
    ```

    And open "**/dynamo_planner/CMakeList.txt**" as a project.  
    Then run main.  

    **B. Not using CLion, just using terminal**  
    Go to MotionPlanning_ws  

    ```bash
    cd ~/MotionPlanning_ws
    ```

    Enter following command  

    ```bash
    catkin_make
    ```

    After compile is done without any error messeges, enter  

    ```bash
    roslaunch husky_gazebo husky_empty_world.launch
    ```

    and  

    ```bash
    rosrun dynamo_planner main
    ```

# Folder description

-   **dynamo_planner**  
    This is a actual source code directory.

    ## How to access to the source code

       Go to "/src" and there exist following files.  
       <center> main.cpp, Scene.cpp, StateSpaces.cpp, DynaMoP.cpp </center>

       **main.cpp** runs total procedure, global planning, local planning and displaying.      

       **Scene.cpp** consist the synthetic scene with obstacles.  

       **StateSpaces.cpp** checks the state validity.  

       **DynaMoP.cpp** contains main planner's implementation. Basically RRT8, but can be modulated by using OMPL's library.   

-   **husky**  
    This directory contains the model(**husky**)'s data. And rnu the simulator.
    ## How to launch simulator
    ```bash
    roslaunch husky_gazebo husky_empty_world.launch
    ```
