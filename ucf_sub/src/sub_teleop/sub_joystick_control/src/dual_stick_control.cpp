#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

class ThrustControl {
  ros::NodeHandle nh_;
  ros::Subscriber trans_sub;
  ros::Subscriber rot_sub;
  ros::Publisher twist_pub;
  
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  geometry_msgs::Twist twistMsg;

public:
  ThrustControl()
  {
    trans_sub = nh_.subscribe("/translate/joy", 1, &ThrustControl::translateCb, this);
    rot_sub = nh_.subscribe("/rotate/joy", 1, &ThrustControl::rotateCb, this);
    
    twist_pub = nh_.advertise<geometry_msgs::Twist>("sub/dualStickControl", 1000);
  }
  void init()
  {
    ros::Rate rate(50);
    while(ros::ok()) {
      //twistMsg.linear = linear;
      //twistMsg.angular = angular;
      twist_pub.publish(twistMsg);
      ros::spinOnce();
      rate.sleep();
    }

  }

  void translateCb(const sensor_msgs::Joy &msg)
  {
    if(sizeof(msg.axes)/sizeof(float) < 3) {
      ROS_INFO("JOYSTICK ERROR");
      return;
    }
    twistMsg.linear.x = msg.axes[0];
    twistMsg.linear.y = msg.axes[1];
    twistMsg.linear.z = msg.axes[2];
    ROS_INFO("TRANSLATION UPDATED");
  }

  void rotateCb(const sensor_msgs::Joy &msg)
  {
    if(sizeof(msg.axes)/sizeof(float) < 3) {
      ROS_INFO("JOYSTICK ERROR: Not enough axes");
      return;
    }
    twistMsg.angular.x = msg.axes[0];
    twistMsg.angular.y = msg.axes[1];
    twistMsg.angular.z = msg.axes[2];
    ROS_INFO("ROTATION UPDATED");
  }
};

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "thrust_control");
    ThrustControl tc;
    tc.init();

    return 0;
  }
