#include <iostream>
#include "ros/ros.h"
#include <ctime>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "timer_node");

    ros::NodeHandle nh;
    ros::Rate loop_rate(4);

	time_t	now;
	tm		*ltm;

    while (ros::ok())
    {
		now = time(0);
		ltm = localtime(&now);

        ROS_INFO_STREAM(ltm->tm_hour << ":" << ltm->tm_min << " " << ltm->tm_mday << "." << ltm->tm_mon + 1 << "." << ltm->tm_year + 1900);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
