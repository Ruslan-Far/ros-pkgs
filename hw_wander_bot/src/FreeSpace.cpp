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

	msg.linear.x = 0.0;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0;
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
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

void FreeSpace::setParamsTargetOrient(int targetIndexStart, int targetIndexEnd, int justIndex, bool flag)
{
	if (targetIndexStart == -1.0 && targetIndexEnd == -1.0)
	{
		ROS_INFO("setParamsTargetOrient INF ЛИБО ЕСТЬ, ЛИБО НЕТ");
		if (!flag) // вообще нет inf
		{
			targetOrient = justIndex;
			ROS_INFO("setParamsTargetOrient NO INF justIndex = %d", justIndex);
		}
		else // все есть inf
		{
			targetOrient = 180.0; // default orientation
			ROS_INFO("setParamsTargetOrient YES INF");
		}
	}
	else
	{
		ROS_INFO("setParamsTargetOrient targetOrient = (targetIndexStart + targetIndexEnd) / 2.0;");
		targetOrient = (targetIndexStart + targetIndexEnd) / 2.0;
		if (targetIndexStart >= targetIndexEnd) // если inf находится под индексом 0 или крайний диапазон с inf заканчивается под индексом 0
		{
			ROS_INFO("setParamsTargetOrient если inf находится под индексом 0");
			targetOrient = targetOrient + 180.0;
			if (targetOrient >= 360.0)
				targetOrient -= 360.0;
		}
	}
	if (targetOrient < 180.0)
	{
		targetOrient = 180.0 - targetOrient;
		directionRotation = false;
		ROS_INFO("setParamsTargetOrient targetOrient = %f", targetOrient);
		ROS_INFO("setParamsTargetOrient directionRotation = false");
	}
	else
	{
		targetOrient = targetOrient - 180.0;
		directionRotation = true;
		ROS_INFO("setParamsTargetOrient targetOrient = %f", targetOrient);
		ROS_INFO("setParamsTargetOrient directionRotation = true");
	}
	flagOrient = true;
	isRotation = true;
	ROS_INFO("setParamsTargetOrient targetIndexStart = %d", targetIndexStart);
	ROS_INFO("setParamsTargetOrient targetIndexEnd = %d", targetIndexEnd);
}

void FreeSpace::findFreeSpace(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int indexStart;
	int indexEnd;
	int targetIndexStart;
	int targetIndexEnd;
	int loopIndexStart;
	int loopIndexEnd;
	int justIndex; // для случая, если вообще не будет inf
	double rangeStart;
	double rangeEnd;
	double maxRangeStart;
	double maxRangeEnd;
	double justMax; // для случая, если вообще не будет inf
	bool isInf;
	bool flag; // если нашли начало диапазона inf

	indexStart = -1;
	indexEnd = -1;
	targetIndexStart = -1;
	targetIndexEnd = -1;
	loopIndexStart = ceil(scan->angle_min / scan->angle_increment);
	loopIndexEnd = ceil(scan->angle_max / scan->angle_increment) + 1;
	justIndex = -1;
	rangeStart = -1.0;
	rangeEnd = -1.0;
	maxRangeStart = -1.0;
	maxRangeEnd = -1.0;
	justMax = -1.0;
	isInf = false;
	flag = false;
	for (int i = loopIndexStart; i < loopIndexEnd; i++)
	{
		isInf = !(scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max);
		if (!isInf && scan->ranges[i] > justMax)
		{
			justIndex = i;
			justMax = scan->ranges[justIndex];
		}
		if (isInf)
		{
			if (!flag)
			{
				indexStart = i - 1;
				if (indexStart == -1)
				{
					for (int j = loopIndexEnd - 1; j > 0; j--)
					{
						if (scan->ranges[j] >= scan->range_min && scan->ranges[j] <= scan->range_max)
						{
							indexStart = j;
							break;
						}
					}
					if (indexStart == -1)
					{
						flag = true;
						break;
					}
					else
						loopIndexEnd = indexStart;
				}
				rangeStart = scan->ranges[indexStart];
				flag = true;
			}
		}
		if (flag && (!isInf || i + 1 == loopIndexEnd))
		{
			if (!isInf)
				indexEnd = i;
			else
				indexEnd = loopIndexEnd % 360;
			rangeEnd = scan->ranges[indexEnd];
			if (rangeStart + rangeEnd > maxRangeStart + maxRangeEnd)
			{
				targetIndexStart = indexStart;
				targetIndexEnd = indexEnd;
				maxRangeStart = rangeStart;
				maxRangeEnd = rangeEnd;
			}
			else if (rangeStart + rangeEnd == maxRangeStart + maxRangeEnd)
			{
				double targetDelta = targetIndexEnd - targetIndexStart;
				double delta = indexEnd - indexStart;

				if (targetDelta < 0)
					targetDelta = 360 + targetDelta;
				if (delta < 0)
					delta = 360 + delta;
				if (delta > targetDelta)
				{
					targetIndexStart = indexStart;
					targetIndexEnd = indexEnd;
				}
			}
			flag = false;
		}
	}
	setParamsTargetOrient(targetIndexStart, targetIndexEnd, justIndex, flag);
	ROS_INFO("findFreeSpace ranges");
	for (int i = 0; i < 360; i++)
		ROS_INFO("[%d] = %f", i, scan->ranges[i]);
}

void FreeSpace::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = ceil((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

	if (flagFirstFreeSpace)
	{
		findFreeSpace(scan);
		flagFirstFreeSpace = false;
	}
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
		else if (isRotation && !flagOrient)
			moveAngular(directionRotation);
        ros::spinOnce();
        rate.sleep();
    }
}
