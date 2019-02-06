#include <fire_fighter/manipulation_planner_node.h>

TaskspacePlanner::TaskspacePlanner(ros::NodeHandle &nh, std::string urdf_param):nh_(nh), wholebodyController_(nh)
{
    state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
    rd_ = RobotDescription::getRobotDescription(nh);

    if (urdf_param == "") {
        urdf_param.assign("/"+rd_->getRobotName()+"/robot_description");
    }

    robot_model_loader::RobotModelLoader robot_model_loader(urdf_param);
    robot_model_ = robot_model_loader.getModel();

    display_publisher_ =
            nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    position_tolerance_  =  0.01;
    angle_tolerance_ = 0.01;
    moveit_robot_state_  = std::shared_ptr<robot_state::RobotState>(new robot_state::RobotState(robot_model_));

    plugin_param_ = "/move_group/planning_plugin";
    loadPlanners();

}

TaskspacePlanner::~TaskspacePlanner(){
    planner_instance.reset();
}

bool TaskspacePlanner::execute(const geometry_msgs::PoseStamped &pose, std::string planning_group)
{      

    planning_interface::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse response_msg;

    state_informer_->getJointStateMessage(req.start_state.joint_state);

    req.group_name = planning_group;

    // configured planning groups are all upper case. right arm begins with R and left arm begins with L
    std::string ee_frame =  planning_group.front() == 'R' ? rd_->getRightEEFrame() : rd_->getLeftEEFrame();

    moveit_msgs::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints(ee_frame, pose, position_tolerance_, angle_tolerance_);
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context =
            planner_instance->getPlanningContext(planning_scene_, req, res_.error_code_);
    context->solve(res_);
    if (res_.error_code_.val != res_.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return false;
    }

    res_.getMessage(response_msg);

    //     Visualize the trajectory
    displayInRviz(response_msg);

    // Robot state is required for generating RobotTrajectory object
    // RobotTrajectoryObject is required for generating creating IterativeParabolicTimeParameterization
    // IterativeParabolicTimeParameterization is required for generating timestamps for each trajectory point
    // Procedure:   create RobotTrajectory object using the RobotModelPtr and planning group name
    //              create RobotState object using the same RobotModelPtr
    //              After planner generates a new trajectory, update the robot state
    //              update the RobotTrajectory
    //              call computeTimeStamps
    //              convert RobotTrajectory back to moveit_msgs_RobotTrajectory



    robot_trajectory::RobotTrajectory robot_traj = robot_trajectory::RobotTrajectory(robot_model_, planning_group);
    moveit_robot_state_->update();
    robot_traj.setRobotTrajectoryMsg(*moveit_robot_state_, response_msg.trajectory.joint_trajectory);
    timeParameterizer.computeTimeStamps(robot_traj);

    robot_traj.getRobotTrajectoryMsg(response_msg.trajectory);

    //    for(size_t i = 0; i < response.trajectory.joint_trajectory.joint_names.size() ; i++){
    //        ROS_INFO("%d: Joint Name %s, number of points %d", i, response.trajectory.joint_trajectory.joint_names.at(i).c_str(), response.trajectory.joint_trajectory.points.size());
    //    }

    /// @todo: move this into calling node
    wholebodyController_.executeTrajectory(response_msg.trajectory);

    return true;
}

double TaskspacePlanner::getPositionTolerance() const
{
    return position_tolerance_;
}

void TaskspacePlanner::setPositionTolerance(const double tolerance_pose)
{
    position_tolerance_ = tolerance_pose;
}

double TaskspacePlanner::getAngleTolerance() const
{
    return angle_tolerance_;
}

void TaskspacePlanner::setAngleTolerance(const double tolerance_angle)
{
    angle_tolerance_ = tolerance_angle;
}

std::string TaskspacePlanner::getPluginParameter() const{
    return plugin_param_;
}

void TaskspacePlanner::setPluginParameter(const std::string &plugin_param){
    plugin_param_ = plugin_param;
    loadPlanners();
}

void TaskspacePlanner::displayInRviz(const moveit_msgs::MotionPlanResponse &response_msg)
{
    ROS_INFO("Visualizing the trajectory");
    display_trajectory_.trajectory_start = response_msg.trajectory_start;
    display_trajectory_.trajectory.push_back(response_msg.trajectory);
    display_publisher_.publish(display_trajectory_);
}


void TaskspacePlanner::loadPlanners()
{
    ROS_INFO("Loading Planners");

    if (!nh_.getParam(plugin_param_, planner_plugin_name))
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

        if (!planner_instance->initialize(robot_model_, nh_.getNamespace()))
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
}
