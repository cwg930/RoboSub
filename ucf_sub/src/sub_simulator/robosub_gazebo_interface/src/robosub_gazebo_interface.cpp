#include <robosub_gazebo_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gate_finder");
    GazeboInterface gi;
    ros::spin();
    return 0;
}

template <typename T> int sgn(T val) {  //http://stackoverflow.com/a/4609795
    return (T(0) < val) - (val < T(0));
}

GazeboInterface::GazeboInterface()
{
    commandSub = nh_.subscribe("/sub/dualStickControl", 1000, &GazeboInterface::commandCb, this);
    modelUpdate = nh_.subscribe("/gazebo/model_states", 1000, &GazeboInterface::stateCb, this);
    gazeboWrenchCaller = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazeboStopCaller = nh_.serviceClient<gazebo_msgs::BodyRequest>("/gazebo/clear_body_wrenches");

    joystickInput.angular.x = 0.0;
    joystickInput.angular.y = 0.0;
    joystickInput.angular.z = 0.0;

    joystickInput.linear.x = 0.0;
    joystickInput.linear.y = 0.0;
    joystickInput.linear.z = 0.0;
}

GazeboInterface::~GazeboInterface()
{

}
void GazeboInterface::stateCb(const gazebo_msgs::ModelStates &msg)
{
    int subIndex = 0;
    if(std::find(msg.name.begin(),msg.name.end(), "ucf_submarine_simple") != msg.name.end()){
        subIndex = std::find(msg.name.begin(),msg.name.end(), "ucf_submarine_simple") - msg.name.begin();
    }
    else {
        return;
    }

    geometry_msgs::Quaternion orientation = msg.pose[subIndex].orientation;
    tf::Quaternion rotWtL(orientation.x, orientation.y, orientation.z, orientation.w);
    //tf::Quaternion rotLtW = rotWtL.inverse();

    tf::Vector3 gravityLocal = tf::quatRotate(rotWtL, tf::Vector3(0,0,-500));
    tf::Vector3 bouyancyLocal = tf::quatRotate(rotWtL, tf::Vector3(0,0,500));
    tf::Vector3 gravityOffset = tf::Vector3(0,0,-0.2);
    tf::Vector3 bouyancyOffset = tf::Vector3(0,0,0.2);

    tf::Vector3 stabilityTorque = gravityOffset.cross(gravityLocal) + bouyancyOffset.cross(bouyancyLocal);
    tf::Vector3 floatiness = gravityLocal+bouyancyLocal;


    gazebo_msgs::BodyRequest stop;
    stop.request.body_name = "ucf_submarine_simple";

    gazebo_msgs::ApplyBodyWrench wrench;
    wrench.request.body_name = "ucf_submarine_simple::body";
    wrench.request.reference_frame = "ucf_submarine_simple::body";

    //You can't deliver maximum thrust and maximum torque on the same axis, so mix things to represent this
    //+x forward, +z down, +y right
    float tFrontUp, tRearUp, tLeftForward, tRightForward, tTopStrafe, tBottomStrafe;

    tFrontUp = std::max(-1.0, std::min(1.0, joystickInput.linear.z + joystickInput.angular.y));
    tRearUp = std::max(-1.0, std::min(1.0, joystickInput.linear.z - joystickInput.angular.y));

    tLeftForward = std::max(-1.0, std::min(1.0, joystickInput.linear.x - joystickInput.angular.z));
    tRightForward = std::max(-1.0, std::min(1.0, joystickInput.linear.x + joystickInput.angular.z));

    tTopStrafe = std::max(-1.0, std::min(1.0, joystickInput.linear.y - joystickInput.angular.x));
    tBottomStrafe = std::max(-1.0, std::min(1.0, joystickInput.linear.y + joystickInput.angular.x));

    wrench.request.wrench.force.x = (tLeftForward + tRightForward) * MAX_THRUST + floatiness.x();
    wrench.request.wrench.force.y = (tTopStrafe + tBottomStrafe) * MAX_THRUST + floatiness.y();
    wrench.request.wrench.force.z = (tFrontUp + tRearUp) * MAX_THRUST + floatiness.z();

    wrench.request.wrench.torque.x = (tBottomStrafe - tTopStrafe) * MAX_TORQUE_ROLL + stabilityTorque.x();
    wrench.request.wrench.torque.y = (tFrontUp - tRearUp) * MAX_TORQUE_PITCH + stabilityTorque.y();
    wrench.request.wrench.torque.z = (tRightForward - tBottomStrafe) * MAX_TORQUE_YAW + stabilityTorque.z();

    wrench.request.duration.sec = 0.01;
    wrench.request.start_time.sec = 0;

    gazeboStopCaller.call(stop);
    gazeboWrenchCaller.call(wrench);

    std::ostringstream oss;
    double x = wrench.request.wrench.torque.x;
    double y = wrench.request.wrench.torque.y;
    double z = wrench.request.wrench.torque.z;
    //oss << "Stability Torque: (" << x << ", " << y << ", " << z << ")";
    //ROS_INFO(oss.str().c_str());
}

void GazeboInterface::commandCb(const geometry_msgs::Twist &msg)
{
    joystickInput = msg;
}
