#include "SnakeMv.h"

SnakeMv::SnakeMv()
{
	isObstacle = true;
	isRotation = false;
	isAfterOddRotation = false;
	flagPosition = false;
	numRotation = 0;
	startPositionX = 0.0;
	startPositionY = 0.0;





	flagOrient = false;
	startOrient = 0.0;

    cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	modelStatesSub = node.subscribe("/gazebo/model_states", 1, &SnakeMv::modelStatesCallback, this);

    scanSub = node.subscribe("/scan", 1, &SnakeMv::scanCallback, this);
}

void SnakeMv::moveLinear()
{
    geometry_msgs::Twist msg;

    msg.linear.x = (-1) * LINEAR_SPEED;
    cmdVelPub.publish(msg);
}

void SnakeMv::moveAngular(bool direction)
{
    geometry_msgs::Twist msg;

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

void SnakeMv::updateRotations()
{
	ROS_INFO("modelStatesCallback isRotation");
	stop();
	if (numRotation % 2 == 0)
		isAfterOddRotation = true;
	numRotation = (numRotation + 1) % 4;
	isRotation = false;
}

void SnakeMv::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	ROS_INFO("curOrient = %f", getCurOrient(modelStates));
	// if (isRotation)
	// {
	// 	double curAngRot = modelStates->pose[2].orientation.z;
	// 	double curAuxAngRot = modelStates->pose[2].orientation.w;

	// 	if (numRotation == 0 && curAngRot < ANGS_ROT[0])
	// 		updateRotations();
	// 	else if (numRotation == 1 && curAuxAngRot < AUX_ANGS_ROT[0])
	// 		updateRotations();
	// 	else if (numRotation == 2 && curAngRot > ANGS_ROT[0])
	// 		updateRotations();
	// 	else if (numRotation == 3 && curAngRot > ANGS_ROT[1])
	// 		updateRotations();
	// }
	if (isRotation)
	{
		initStartOrient(modelStates);
		double curOrient = getCurOrient(modelStates);
		double deltaOrient = abs(abs(curOrient) - abs(startOrient));

		if (curOrient >= 0 && startOrient <= 0 || curOrient <= 0 && startOrient >= 0)
			deltaOrient = 360.0 - deltaOrient;
		if (numRotation == 0 && deltaOrient >= ORIENTS[0])
			updateRotations();
		else if (numRotation == 1 && deltaOrient >= ORIENTS[0])
			updateRotations();
		else if (numRotation == 2 && deltaOrient >= ORIENTS[0])
			updateRotations();
		else if (numRotation == 3 && deltaOrient >= ORIENTS[0])
			updateRotations();
	}
	else if (isAfterOddRotation)
	{
		initStartPositionXY(modelStates);
		double currentPositionX = modelStates->pose[2].position.x;
		double currentPositionY = modelStates->pose[2].position.y;
		double currentDistShortSide = sqrt(pow(currentPositionX - startPositionX, 2.0) + pow(currentPositionY - startPositionY, 2.0));
		
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
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
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
			isRotation = true;
			isAfterOddRotation = false;
			flagPosition = true;
			flagOrient = true;
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
		else if (isRotation)
		{
			if (numRotation < 2)
				moveAngular(false);
			else
				moveAngular(true);
		}
		// moveAngular(true);
        ros::spinOnce();
        rate.sleep();
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

