#include <fire_fighter/manipulation_planner_node.h>

//ManipulationPlannerNode::ManipulationPlannerNode(ros::NodeHandle &nh)
//{
//    int i = 0;
//}

int ManipulationPlannerNode::execute(std::string PLANNING_GROUP, geometry_msgs::PoseStamped pose, std::string link_name)
{
    robot_model_loader::RobotModelLoader robot_model_loader("/atlas/robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // Robot state is required for generating RobotTrajectory object
    // RobotTrajectoryObject is required for generating creating IterativeParabolicTimeParameterization
    // IterativeParabolicTimeParameterization is required for generating timestamps for each trajectory point
    // Procedure:   create RobotTrajectory object using the RobotModelPtr and planning group name
    //              create RobotState object using the same RobotModelPtr
    //              After planner generates a new trajectory, update the robot state
    //              update the RobotTrajectory
    //              call computeTimeStamps
    //              convert RobotTrajectory back to moveit_msgs_RobotTrajectory


    robot_model::RobotState moveitRobotState = robot_state::RobotState(robot_model);
    robot_trajectory::RobotTrajectory robot_traj = robot_trajectory::RobotTrajectory(robot_model, PLANNING_GROUP);

    trajectory_processing::IterativeParabolicTimeParameterization timeParameterizer;




    if (!nh.getParam("/move_group/planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, nh.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);


    robotStateInformer->getJointStateMessage(req.start_state.joint_state);

    req.group_name = PLANNING_GROUP;

//    geometry_msgs::PoseStamped pose;
//    pose.header.frame_id = "pelvis";
//    pose.pose.position.x = 0.2;
//    pose.pose.position.y = 0.4;
//    pose.pose.position.z = 0.2;
//    pose.pose.orientation.w = 1.0;

    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints(link_name, pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);


    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    ros::Publisher display_publisher =
        nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    moveitRobotState.update();
    robot_traj.setRobotTrajectoryMsg(moveitRobotState, response.trajectory.joint_trajectory);
    timeParameterizer.computeTimeStamps(robot_traj);

    robot_traj.getRobotTrajectoryMsg(response.trajectory);

    for(size_t i = 0; i < response.trajectory.joint_trajectory.joint_names.size() ; i++){
        ROS_INFO("%d: Joint Name %s, number of points %d", i, response.trajectory.joint_trajectory.joint_names.at(i).c_str(), response.trajectory.joint_trajectory.points.size());
    }

    WholebodyControlInterface wholeBody(nh);
    wholeBody.executeTrajectory(RobotSide::LEFT,response.trajectory);


    ros::Duration(10).sleep();

    planner_instance.reset();

    return 0;
}
