#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

static int start = 0;

static double x_lin_pose_start;
static double z_ang_pose_start;

static double x_lin_pose_current;
static double z_ang_pose_current;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	if (start == 0) {
		x_lin_pose_start = msg->x;
		z_ang_pose_start = msg->theta;
		start = 1;
	}
	x_lin_pose_current = msg->x;
	z_ang_pose_current = msg->theta;
	ROS_INFO("x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
}

void	update_pose_start()
{
	x_lin_pose_start = x_lin_pose_current;
	z_ang_pose_start = z_ang_pose_current;
}

void	reset_speed(geometry_msgs::Twist *msg)
{
	msg->linear.x = 0;
    msg->linear.y = 0;
	msg->linear.z = 0;
	msg->angular.x = 0;
	msg->angular.y = 0;
	msg->angular.z = 0;
}

int main(int argc, char **argv)
{
    const double FORWARD_SPEED_MPS = 0.5;

	const double X_DISTANCE = 3.0;

	const double Z_ROTATION = 3.0 * M_PI / 4.0;

    ros::init(argc, argv, "digit_turtle");

    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    ros::Subscriber sub = node.subscribe("turtle1/pose", 10, poseCallback);

    geometry_msgs::Twist msg;

	reset_speed(&msg);

    ros::Rate rate(100);
	
    ROS_INFO("Start");

    while (ros::ok())
    {
		// 1
		if (start == 1 && (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE))
		{
    		msg.linear.x = FORWARD_SPEED_MPS;
		}
		else if (start == 1 && !(abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 2;
		}

		// 2
		if (start == 2 && (abs(z_ang_pose_current - z_ang_pose_start) < Z_ROTATION))
		{
			msg.angular.z = -FORWARD_SPEED_MPS;
		}
		else if (start == 2 && !(abs(z_ang_pose_current - z_ang_pose_start) < Z_ROTATION))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 3;
		}

		// 3
		if (start == 3 && (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE))
		{
			msg.linear.x = FORWARD_SPEED_MPS;
		}
		else if (start == 3 && !(abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 4;
		}

		// 4
		if (start == 4 && (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 2.0))
		{
			msg.linear.x = -FORWARD_SPEED_MPS;
		}
		else if (start == 4 && !(abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 2.0))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 5;
		}

		// 5
		if (start == 5 && (abs(z_ang_pose_current - z_ang_pose_start) < M_PI - Z_ROTATION))
		{
			msg.angular.z = -FORWARD_SPEED_MPS;
		}
		else if (start == 5 && !(abs(z_ang_pose_current - z_ang_pose_start) < M_PI - Z_ROTATION))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 6;
		}

		// 6
		if (start == 6 && (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 3.0))
		{
			msg.linear.x = FORWARD_SPEED_MPS;
		}
		else if (start == 6 && !(abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 3.0))
		{
			update_pose_start();
			reset_speed(&msg);
			start = 7;
		}

		// 7
		if (start == 7 && (abs(x_lin_pose_current - x_lin_pose_start) < 2.0 * X_DISTANCE / 3.0))
		{
			msg.linear.x = -FORWARD_SPEED_MPS;
		}
		else if (start == 7 && !(abs(x_lin_pose_current - x_lin_pose_start) < 2.0 * X_DISTANCE / 3.0))
		{
			update_pose_start();
			reset_speed(&msg);
			break;
		}
    	pub.publish(msg);
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("End");

    return 0;
}
