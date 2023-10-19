#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
using namespace std;

static int a = 0;
static double poseStart;
static double poseCurrent;

void poseCallback(const turtlesim::PoseConstPtr& msg) // Topic messages callback
{
  if (a==0) {
  	poseStart = msg->x;
  	a=1;
  }
  poseCurrent = msg->x;
  ROS_INFO("x: %.2f, y: %.2f", msg->x, msg->y);
}

int main(int argc, char ** argv)
{
    const double FORWARD_SPEED_MPS = 1;

    string robot_name = string(argv[1]);

    ros::init(argc, argv, "move_turtle");   //Initialize the node
    ros::NodeHandle node;

    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // A listener for pose
    ros::Subscriber sub = node.subscribe(robot_name + "/pose", 10, poseCallback);

    // Drive forward at a given speed. The robot points up the x-axis.
    // The default constuctor will set all commands to 0.
    geometry_msgs::Twist msg;
    // msg.linear.x = FORWARD_SPEED_MPS;
    msg.linear.y = FORWARD_SPEED_MPS;
	// msg.angular.z = FORWARD_SPEED_MPS;

    // Loop at 10Hz, publishing movement conmmands until we shut down.
    ros::Rate rate(1);
    ROS_INFO("Starting to move forward");

    // while (ros::ok() && (poseCurrent - poseStart < 1.0))
    // {
    //   pub.publish(msg);
	// //   ROS_INFO("hi");
    //   ros::spinOnce(); // Allow processing of incoming messages
    //   rate.sleep();
    // }

	// while (ros::ok())
    // {
    //   pub.publish(msg);
	//   ROS_INFO("hi");
    //   ros::spinOnce(); // Allow processing of incoming messages
    //   rate.sleep();
    // }

	// while (ros::ok() && (poseCurrent - poseStart < 0))
    // {
    //   pub.publish(msg);
    //   ros::spinOnce(); // Allow processing of incoming messages
    //   rate.sleep();
    // }
	// while (ros::ok()) {}

	int i = 0;
	// while (i < 2)
	// {
		pub.publish(msg);
		ROS_INFO("------------");
		pub.publish(msg);
		ROS_INFO("------------");
      	// ros::spinOnce();
      	// rate.sleep();
		// i++;
	// }

    return 0;
}