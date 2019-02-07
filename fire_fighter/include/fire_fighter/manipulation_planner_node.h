#ifndef TASKSPACE_PLANNER_H
#define TASKSPACE_PLANNER_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

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
#include <tough_common/robot_description.h>
#include <tough_common/tough_common_names.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_common/robot_description.h>

class TaskspacePlanner
{
public:
    TaskspacePlanner(ros::NodeHandle &nh, std::string urdf_param="");
    ~TaskspacePlanner();
    bool execute(geometry_msgs::PoseStamped &pose, std::string planning_group = TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP);

    double getPositionTolerance() const;
    void setPositionTolerance(const double position_tolerance);

    double getAngleTolerance() const;
    void setAngleTolerance(const double tolerance_angle);

    std::string getPluginParameter() const;
    void setPluginParameter(const std::string &plugin_param);
private:
    ros::NodeHandle nh_;
    ros::Publisher display_publisher_;

    std::shared_ptr<robot_model::RobotState>  moveit_robot_state_ ;
    std::string plugin_param_;

    double planning_time_;
    int num_planning_attempts_;

    moveit_msgs::DisplayTrajectory display_trajectory_;
//    robot_model::RobotState moveit_robot_state_  ;
//    robot_trajectory::RobotTrajectory robot_traj_;
    robot_model::RobotModelPtr robot_model_;

    WholebodyControlInterface wholebodyController_;
    std::string planner_plugin_name;
    planning_interface::PlannerManagerPtr planner_instance;
    planning_scene::PlanningScenePtr planning_scene_;


    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::MotionPlanResponse res_;

    RobotStateInformer *state_informer_;
    RobotDescription *rd_;
    //    planning_scene::PlanningScenePtr planning_scene;
    trajectory_processing::IterativeParabolicTimeParameterization timeParameterizer;
    void fixEEOrientation(const RobotSide side, geometry_msgs::Quaternion &orientation);

    double position_tolerance_;
    double angle_tolerance_;

    void displayInRviz(const moveit_msgs::MotionPlanResponse &response_msg);
    void loadPlanners();
};


#endif // TASKSPACE_PLANNER_H
