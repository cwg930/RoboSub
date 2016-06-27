#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sub_move_controller/MoveCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

class MoveCommandAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sub_move_controller::MoveCommandAction> as_;
  std::string action_name_;
  sub_move_controller::MoveCommandFeedback feedback_;
  sub_move_controller::MoveCommandResult result_;
  robot_model::RobotModelPtr robot_model;
  planning_scene::PlanningScenePtr *planning_scene;
  planning_interface::PlannerManagerPtr planner_instance;

public:

  MoveCommandAction(std::string name):
    as_(nh_, name, boost::bind(&MoveCommandAction::executeCb, this, _1), false),
    action_name_(name)
  {
    //load robot description URDF/SRDF from ros parameter server
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model = robot_model_loader.getModel();
    planning_scene = new planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS param server, and then load the planner
    // making sure to catch all exceptions.
    if (!nh_.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, nh_.getNamespace()))
	ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0 ; i < classes.size() ; ++i)
	ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
		       << "Available plugins: " << ss.str());
    }
    as_.start();
  }

  ~MoveCommandAction()
  {
  }

  void executeCb(const sub_move_controller::MoveCommandGoalConstPtr& goal)
  {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    
    as_.setSucceeded(result_);
  }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_move_server");
    MoveCommandAction moveServer(ros::this_node::getName());
    ros::spin();
    return 0;
}
