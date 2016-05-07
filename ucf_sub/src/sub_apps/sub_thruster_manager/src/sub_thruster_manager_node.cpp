#include "blue_robotics_t200/t200_thruster.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;

    std::vector<T200Thruster> thrusters;

public:
    ThrusterManager()
    {
        command_subscriber = nh_.subscribe("/thrustercommands", 1000, &ThrusterManager::thrusterCb, this);

        thrusters.reserve(6);
        thrusters[0] = T200Thruster(2, 0x2D);
        thrusters[1] = T200Thruster(2, 0x2E);
    }

    void init()
    {
        ros::Rate rate(50);
        while(ros::ok()) {
            //Publish diagnostic data here
            ros::spinOnce();
            rate.sleep();
        }
    }

    void thrusterCb(const geometry_msgs::Twist &msg)
    {
        //copy data out of the message so we can modify as necessary
        float angularX, angularY, angularZ, linearX, linearY, linearZ;

        angularX = msg.angular.x;
        angularY = msg.angular.y;
        angularZ = msg.angular.z;

        linearX = msg.linear.x;
        linearY = msg.linear.y;
        linearZ = msg.linear.z;

        //normalize linear/angular pairs that depend on the same thruster pairs
        /*
          We need this because trying to apply a tranlation force of +/-1 and a torque of +/-1 isnt possible
          with our thruster setup, so we normalize in order to approximate whatever mix the controller wants
        */

        if(magnitude(linearZ, angularY) > 1.0)
        {
            linearZ = linearZ / magnitude(linearZ, angularY);
            angularY = angularY /  magnitude(linearZ, angularY);
        }

        if(magnitude(linearX, angularZ) > 1.0)
        {
            linearX = linearX / magnitude(linearX, angularZ);
            angularZ = angularZ /  magnitude(linearX, angularZ);
        }

        if(magnitude(linearY, angularX) > 1.0)
        {
            linearY = linearY / magnitude(linearY, angularX);
            angularX = angularX /  magnitude(linearY, angularX);
        }

        //mix the twist message, limits for extra safety
        float tFrontUp, tRearUp, tLeftForward, tRightForward, tTopStrafe, tBottomStrafe;

        tFrontUp = std::max(-1.0f, std::min(1.0f, linearZ + angularY));
        tRearUp = std::max(-1.0f, std::min(1.0f, linearZ  - angularY));

        tLeftForward = std::max(-1.0f, std::min(1.0f, linearX - angularZ));
        tRightForward = std::max(-1.0f, std::min(1.0f, linearX + angularZ));

        tTopStrafe = std::max(-1.0f, std::min(1.0f, linearY - angularX));
        tBottomStrafe = std::max(-1.0f, std::min(1.0f, linearY + angularX));

        //command the thrusters to the desired velocity
        thrusters[0].setVelocityRatio(abs(tLeftForward), (tLeftForward > 0.0f) ? T200ThrusterDirections::Forward : T200ThrusterDirections::Reverse);
        thrusters[1].setVelocityRatio(abs(tRightForward), tRightForward > 0.0f ? T200ThrusterDirections::Forward : T200ThrusterDirections::Reverse);

    }
    float magnitude(float x, float y) //return the magnitude of a 2d vector
    {
        return sqrt(x*x + y*y);
    }
};

int main(int argc, char** argv)
{
    return 0;
}
