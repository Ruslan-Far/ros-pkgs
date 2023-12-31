#include "SnakeMv.h"

SnakeMv::SnakeMv()
{
	isObstacle = true;
	isRotation = false;
	isAfterOddRotation = false;
	flagPosition = false;
	flagOrient = false;
	numRotation = 0;
	startPositionX = 0.0;
	startPositionY = 0.0;
	startOrient = 0.0;

    cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	modelStatesSub = node.subscribe("/gazebo/model_states", 1, &SnakeMv::modelStatesCallback, this);

    scanSub = node.subscribe("/scan", 1, &SnakeMv::scanCallback, this);
}

void SnakeMv::moveLinear()
{
    geometry_msgs::Twist msg;

    msg.linear.x = (-1) * LINEAR_SPEED;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0;
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
	msg.angular.z = 0.0;
    cmdVelPub.publish(msg);
}

void SnakeMv::moveAngular(bool direction)
{
    geometry_msgs::Twist msg;

	msg.linear.x = 0.0;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0;
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
	if (direction)
    	msg.angular.z = ANGULAR_SPEED;
	else
		msg.angular.z = (-1) * ANGULAR_SPEED;
    cmdVelPub.publish(msg);
}

void SnakeMv::stop()
{
    ROS_INFO("Stop!");
	geometry_msgs::Twist msg;

	msg.linear.x = 0.0;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0;
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
	msg.angular.z = 0.0;
	cmdVelPub.publish(msg);
	sleep(2);
}

void SnakeMv::initStartPositionXY(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (flagPosition)
	{
		startPositionX = modelStates->pose[2].position.x;
		startPositionY = modelStates->pose[2].position.y;
		flagPosition = false;
	}
}

void SnakeMv::initStartOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (flagOrient)
	{
		startOrient = getCurOrient(modelStates);
		flagOrient = false;
	}
}

double SnakeMv::getCurOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	double z = modelStates->pose[2].orientation.z;
	double w = modelStates->pose[2].orientation.w;
	double curOrient;

	if (z <= 0 && w >= 0 || z > 0 && w < 0)
	{
		curOrient = abs(2.0 * asin(z)) * 180.0 / M_PI;
		if (z > 0 && w < 0)
			curOrient = -curOrient;
	}
	else
	{
		curOrient = (M_PI + abs(2.0 * asin(w))) * 180.0 / M_PI;
		if (z > 0 && w >= 0)
			curOrient = -curOrient;
	}
	return curOrient;
}

void SnakeMv::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (isRotation)
	{
		initStartOrient(modelStates);
		double curOrient = getCurOrient(modelStates);
		double deltaOrient = abs(abs(curOrient) - abs(startOrient));

		if (curOrient >= 0 && startOrient <= 0 || curOrient <= 0 && startOrient >= 0)
			deltaOrient = 360.0 - deltaOrient;
		ROS_INFO("modelStatesCallback curOrient = %f", curOrient);
		ROS_INFO("modelStatesCallback deltaOrient = %f", deltaOrient);
		if (deltaOrient >= ORIENT)
		{
			ROS_INFO("modelStatesCallback isRotation");
			stop();
			if (numRotation % 2 == 0)
				isAfterOddRotation = true;
			numRotation = (numRotation + 1) % 4;
			isRotation = false;
		}
	}
	if (isAfterOddRotation)
	{
		initStartPositionXY(modelStates);
		double currentPositionX = modelStates->pose[2].position.x;
		double currentPositionY = modelStates->pose[2].position.y;
		double currentDistShortSide = sqrt(pow(currentPositionX - startPositionX, 2.0) + pow(currentPositionY - startPositionY, 2.0));
		
		ROS_INFO("modelStatesCallback currentDistShortSide = %f", currentDistShortSide);
		if (currentDistShortSide >= DIST_SHORT_SIDE)
		{
			ROS_INFO("modelStatesCallback isAfterOddRotation");
			stop();
			flagOrient = true;
			isRotation = true;
			isAfterOddRotation = false;
		}
	}
}

void SnakeMv::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = ceil((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex; currIndex <= maxIndex; currIndex++)
	{
        if (scan->ranges[currIndex] <= MIN_DIST_FROM_OBSTACLE)
		{
            isObstacleInFront = true;
            break;
        }
    }
    if (isObstacleInFront)
	{
		isObstacle = true;
		if (!isRotation)
		{
			ROS_INFO("scanCallback isObstacle");
			stop();
			flagOrient = true;
			isRotation = true;
			isAfterOddRotation = false;
			flagPosition = true;
			ROS_INFO("scanCallback ranges");
			for (int i = 0; i < 360; i++)
				ROS_INFO("[%d] = %f", i, scan->ranges[i]);
		}
    }
	else
		isObstacle = false;
}

void SnakeMv::startMoving()
{
    ros::Rate rate(5000);

    ROS_INFO("Start moving");
    while (ros::ok())
	{
		if (!isObstacle && !isRotation)
        	moveLinear();
		else if (isRotation && !flagOrient)
		{
			if (numRotation < 2)
				moveAngular(false);
			else
				moveAngular(true);
		}
        ros::spinOnce();
        rate.sleep();
    }
}
