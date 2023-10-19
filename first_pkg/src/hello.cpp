/*
 * hello.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: lirs
 */

#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Hello"); // инициализация ноды

    ros::NodeHandle nh; // создание точки входа
    ros::Rate loop_rate(10); // частота

    int count = 0;
    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
        ROS_INFO_STREAM("hello world ++++++++++++++++++++++++++++++++++++++ ! " << count); // print to the screen

        ros::spinOnce(); // Allow ROS to process incoming messages
        loop_rate.sleep(); // Sleep for the rest of the cycle
        count++;
    }
    return 0;
}

