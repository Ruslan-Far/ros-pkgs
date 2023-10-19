#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char ** argv)
{
    const double FORWARD_SPEED_MPS = 0.5;

    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_MPS;

    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");

    while (ros::ok())
    {
      pub.publish(msg);
      rate.sleep();
    }

    return 0;
}
