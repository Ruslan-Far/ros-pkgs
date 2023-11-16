#include "SnakeMv.h"

SnakeMv::SnakeMv()
{
    isObstacle = false;

    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	commandSub = node.subscribe("/pose", 10, &SnakeMv::poseCallback, this);

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
    	msg.angle.z = FORWARD_SPEED;
	else
		msg.angle.z = (-1) * FORWARD_SPEED;
    commandPub.publish(msg);
}

void SnakeMv::stop() {
    ROS_INFO("Stop!");
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angle.z = 0.0;
	commandPub.publish(msg);
}

void abstrPoseCallback()
{
	if (isObstacle)
	{
		if (rotZ == 90)
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
		if (distX > 0.5)
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
		// keepMoving = false;
    }
}

void SnakeMv::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    while (ros::ok()) {
		if (!isObstacle)
        	moveForward();
		else
		{
			if (numRotate < 2)
				moveAngle(false);
			else
				moveAngle(true);
		}
        ros::spinOnce();
        rate.sleep();
    }
}
