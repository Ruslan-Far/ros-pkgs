#include "FreeSpace.h"

FreeSpace::FreeSpace()
{
	isObstacle = true;
	isRotation = false;
	flagOrient = false;
	flagFirstFreeSpace = true;
	directionRotation = false;
	startOrient = 0.0;
	targetOrient = 0.0;

    cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	modelStatesSub = node.subscribe("/gazebo/model_states", 1, &FreeSpace::modelStatesCallback, this);

    scanSub = node.subscribe("/scan", 1, &FreeSpace::scanCallback, this);
}

void FreeSpace::moveLinear()
{
    geometry_msgs::Twist msg;

    msg.linear.x = (-1) * LINEAR_SPEED;
    cmdVelPub.publish(msg);
}

void FreeSpace::moveAngular(bool direction)
{
    geometry_msgs::Twist msg;

	if (direction)
    	msg.angular.z = ANGULAR_SPEED;
	else
		msg.angular.z = (-1) * ANGULAR_SPEED;
    cmdVelPub.publish(msg);
}

void FreeSpace::stop()
{
    ROS_INFO("Stop!");
	geometry_msgs::Twist msg;

	msg.linear.x = 0.0;// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	msg.angular.z = 0.0;
	cmdVelPub.publish(msg);
	sleep(2);
}

void FreeSpace::initStartOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (flagOrient)
	{
		startOrient = getCurOrient(modelStates);
		flagOrient = false;
	}
}

double FreeSpace::getCurOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
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

void FreeSpace::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	ROS_INFO("curOrient = %f", getCurOrient(modelStates));
	if (isRotation)
	{
		initStartOrient(modelStates);
		double curOrient = getCurOrient(modelStates);
		double deltaOrient = abs(abs(curOrient) - abs(startOrient));

		if (curOrient >= 0 && startOrient <= 0 || curOrient <= 0 && startOrient >= 0)
			deltaOrient = 360.0 - deltaOrient;
		if (deltaOrient >= targetOrient)
		{
			ROS_INFO("modelStatesCallback isRotation");
			stop();
			isRotation = false;
		}
	}
}

void FreeSpace::findFreeSpace(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int indexStart;
	int indexEnd;
	int targetIndexStart;
	int targetIndexEnd;
	int loopIndexStart;
	int loopIndexEnd;
	int justIndex;
	double rangeStart;
	double rangeEnd;
	double maxRangeStart;
	double maxRangeEnd;
	double justMax;
	bool flag;

	loopIndexStart = ceil(scan->angle_min / scan->angle_increment);
	loopIndexEnd = floor(2.0 * M_PI / scan->angle_increment);
	rangeStart = -1.0;
	rangeEnd = -1.0;
	maxRangeStart = -1.0;
	maxRangeEnd = -1.0;
	justMax = -1.0;
	flag = true;
	for (int i = loopIndexStart; i < loopIndexEnd; i++)
	{
		if (ranges[i] > justMax)
		{
			justIndex = i;
			justMax = ranges[justIndex];
		}
		if (ranges[i] == nan)
		{
			if (flag)
			{
				indexStart = i - 1;
				rangeStart = scan->ranges[indexStart];
				flag = false;
			}
		}
		else if (!flag)
		{
			indexEnd = i;
			rangeEnd = scan->ranges[indexEnd];
			if (rangeStart > maxRangeStart && rangeEnd > maxRangeEnd)
			{
				targetIndexStart = indexStart;
				targetIndexEnd = indexEnd;
				maxRangeStart = rangeStart;
				maxRangeEnd = rangeEnd;
			}
			flag = true;
		}
	}
	if (maxRangeStart == -1.0 || maxRangeEnd = -1.0)
		targetOrient = justIndex;
	else
		targetOrient = (targetIndexStart + targetIndexEnd) / 2.0;
	if (targetOrient < 180.0)
	{
		targetOrient = 180.0 - targetOrient;
		directionRotation = true;
	}
	else
	{
		targetOrient = 180.0 - (360.0 - targetOrient);
		directionRotation = false;
	}
	flagOrient = true;
	isRotation = true;
}

void FreeSpace::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	if (flagFirstFreeSpace)
	{
		findFreeSpace(scan);
		flagFirstFreeSpace = false;
	}
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
			findFreeSpace(scan);
		}
    }
	else
		isObstacle = false;
}

void FreeSpace::startMoving()
{
    ros::Rate rate(5000);

    ROS_INFO("Start moving");
    while (ros::ok())
	{
		if (!isObstacle && !isRotation)
        	moveLinear();
		else if (isRotation)
			moveAngular(directionRotation);
        ros::spinOnce();
        rate.sleep();
    }
}
