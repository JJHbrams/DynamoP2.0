/* Author: Mincheul Kang */

#ifndef DYNAMOP_SCENE_H
#define DYNAMOP_SCENE_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

class Scene
{
public:
    Scene(planning_scene::PlanningScenePtr& planning_scene, std::string frameID);
    void addCollisionObjects();
    void addCollisionObjects(std::vector<moveit_msgs::CollisionObject> tmp_collision_objects);
    void updateCollisionScene();
    moveit_msgs::CollisionObject addBox(std::string name,
                                        double x, double y, double z,
                                        double d_x, double d_y, double d_z,
                                        double yaw);

protected:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::string frameID_;
    planning_scene::PlanningScenePtr planning_scene_;
};

typedef std::shared_ptr<Scene> ScenePtr;

#endif
