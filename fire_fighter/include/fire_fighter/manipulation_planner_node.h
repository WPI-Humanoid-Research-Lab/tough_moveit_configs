#ifndef MANIPULATION_PLANNER_NODE_H
#define MANIPULATION_PLANNER_NODE_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tough_common/robot_state.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_common/robot_description.h>

class ManipulationPlannerNode
{
public:
    ManipulationPlannerNode(ros::NodeHandle &nh);
    int execute(geometry_msgs::PoseStamped pose, std::string PLANNING_GROUP, std::string link_name);

private:
    ros::NodeHandle nh_;
    robot_model::RobotModelPtr robot_model;
    std::string planner_plugin_name;
    planning_interface::PlannerManagerPtr planner_instance;
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    moveit_msgs::MotionPlanResponse response;
    RobotStateInformer *robotStateInformer;
//    planning_scene::PlanningScenePtr planning_scene;
    trajectory_processing::IterativeParabolicTimeParameterization timeParameterizer;

    std::string PLANNING_GROUP = "L_PELVIS_PALM_10DOF";
    std::string link_name = "l_palm";

    void displayInRviz(moveit_msgs::MotionPlanResponse response);
    void publishToWholeBodyTrajectory(moveit_msgs::MotionPlanResponse response, std::string PLANNING_GROUP);
    void loadPlanners();
};


#endif // MANIPULATION_PLANNER_NODE_H
