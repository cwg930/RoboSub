#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sub_move_controller/MoveCommandAction.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_movecontroller");

  actionlib::SimpleActionClient<sub_move_controller::MoveCommandAction> ac("sub_move_server", true);

  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal");
  sub_move_controller::MoveCommandGoal goal;
  geometry_msgs::Vector3 t;
  geometry_msgs::Vector3 r;
  t.x = 0;
  t.y = 0;
  t.z = 0;
  r.x = 0;
  r.y = 0;
  r.z = 0;  
  ac.sendGoal(goal);
}
