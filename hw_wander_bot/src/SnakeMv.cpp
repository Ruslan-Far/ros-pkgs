#include "SnakeMv.h"

SnakeMv::SnakeMv()
{
    isObstacle = false;

    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	modelStatesSub = node.subscribe("/gazebo/model_states", 10, &SnakeMv::modelStatesCallback, this);

    laserSub = node.subscribe("/scan", 1, &SnakeMv::scanCallback, this);
}

void SnakeMv::moveForward() {
    geometry_msgs::Twist msg;
    msg.linear.x = (-1) * FORWARD_SPEED;
    commandPub.publish(msg);
}

void SnakeMv::moveAngle(bool direction) {
    geometry_msgs::Twist msg;
	if (direction)
    	msg.angular.z = FORWARD_SPEED;
	else
		msg.angular.z = (-1) * FORWARD_SPEED;
    commandPub.publish(msg);
}

void SnakeMv::stop() {
    ROS_INFO("Stop!");
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	commandPub.publish(msg);
}

void SnakeMv::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (isObstacle)
	{
		double rotZ = modelStates->pose[2].orientation.z;
		ROS_INFO("rotZ = %f", rotZ);
		if (abs(rotZ) >= 0.707)
		{
			stop();
			if (numRotate % 2 == 0)
				isAfterOddRotate = true;
			numRotate = (numRotate + 1) % 4;
			isObstacle = false;
		}
	}
	else if (isAfterOddRotate)
	{
		double distX = modelStates->pose[2].position.x;
		ROS_INFO("distX = %f", distX);
		if (distX > LENGTH_SHORT_SIDE)
		{
			stop();
			isObstacle = true;
			isAfterOddRotate = false;
		}
	}
}

void SnakeMv::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;

    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleInFront = true;
            break;
        }
    }

    if (isObstacleInFront) {
		if (!isObstacle)
		{
			stop();
			isObstacle = true;
			isAfterOddRotate = false;
		}
    }
}

void SnakeMv::startMoving()
{
    ros::Rate rate(300);
    ROS_INFO("Start moving");

    while (ros::ok()) {
		if (!isObstacle)
        	moveForward();
		else
		{
			if (numRotate < 2)
				moveAngle(true);
			else
				moveAngle(false);
		}
        ros::spinOnce();
        rate.sleep();
    }
}
