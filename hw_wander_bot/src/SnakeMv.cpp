#include "SnakeMv.h"

SnakeMv::SnakeMv()
{

	isObstacle = true;
	isRotation = false;
	isAfterOddRotation = false;
	flagTime = true;
	numRotation = 0;
	startTime = 0.0;

    cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    setModelStatePub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

	modelStatesSub = node.subscribe("/gazebo/model_states", 10, &SnakeMv::modelStatesCallback, this);

    scanSub = node.subscribe("/scan", 1, &SnakeMv::scanCallback, this);

	clockSub = node.subscribe("/clock", 10, &SnakeMv::clockCallback, this);

}

void SnakeMv::moveLinear() {
    geometry_msgs::Twist msg;
    msg.linear.x = (-1) * LINEAR_SPEED;
    cmdVelPub.publish(msg);
}

void SnakeMv::moveAngular(bool direction) {
    geometry_msgs::Twist msg;
	if (direction)
    	msg.angular.z = ANGULAR_SPEED;
	else
		msg.angular.z = (-1) * ANGULAR_SPEED;
    cmdVelPub.publish(msg);
}

void SnakeMv::stop() {
    ROS_INFO("Stop!");
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	cmdVelPub.publish(msg);
	// sleep(2);
}

void SnakeMv::setModelState()
{
	// gazebo_msgs::ModelState modelState;
	// modelState.model_name = "turtlebot3";
	// modelState.reference_frame = "turtlebot3";
	// modelState.pose.orientation.z = 0.707;
	// modelState.twist.angular.z = SPEED;
	// setModelStatePub.publish(modelState);
	// ROS_INFO("))))))))))))))))))))))))))))))))))");
}

void SnakeMv::setModelState2()
{
	// gazebo_msgs::ModelState modelState;
	// // modelState.model_name = "turtlebot3";
	// // modelState.reference_frame = "turtlebot3";
	// // modelState.pose.orientation.z = 0.707;
	// modelState.twist.angular.z = FORWARD_SPEED;
	// setModelStatePub.publish(modelState);
	// ROS_INFO("))))))))))))))))))))))))))))))))))");
}

void SnakeMv::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
	if (isRotation)
	{
		ROS_INFO("currentAngleRotation = %f", modelStates->pose[2].orientation.z);
		ROS_INFO("WcurrentAngleRotation = %f", modelStates->pose[2].orientation.w);
		double currentAngleRotation = abs(modelStates->pose[2].orientation.z);
		if (numRotation > 1)
			currentAngleRotation = -currentAngleRotation;
		if (currentAngleRotation > anglesRotation[numRotation])
		{
			ROS_INFO("modelStatesCallback isRotation");
			stop();
			if (numRotation % 2 == 0)
				isAfterOddRotation = true;
			numRotation = (numRotation + 1) % 4;
			isRotation = false;
			flagTime = true;
		}
	}
	// if (isObstacle)
	// {
	// 	double rotZ = modelStates->pose[2].orientation.z;
	// 	ROS_INFO("rotZ = %f", rotZ);
	// 	if (abs(rotZ) >= 0.707)
	// 	{
	// 		stop();
	// 		if (numRotation % 2 == 0)
	// 			isAfterOddRotation = true;
	// 		numRotation = (numRotation + 1) % 4;
	// 		isObstacle = false;
	// 	}
	// }
	// else if (isAfterOddRotation)
	// {
	// 	double distX = modelStates->pose[2].position.x;
	// 	ROS_INFO("distX = %f", distX);
	// 	if (distX > DIST_SHORT_SIDE)
	// 	{
	// 		stop();
	// 		isObstacle = true;
	// 		isAfterOddRotation = false;
	// 	}
	// }
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
		isObstacle = true;
		if (!isRotation)
		{
			ROS_INFO("scanCallback isObstacle");
			stop();
			isRotation = true;
			isAfterOddRotation = false;
			flagTime = true;
		}
    }
	else
		isObstacle = false;
}

double SnakeMv::getCurrentTime(const rosgraph_msgs::Clock::ConstPtr& clock)
{
	return clock->clock.sec + clock->clock.nsec / 100000000.0;
}

void SnakeMv::initStartTime(const rosgraph_msgs::Clock::ConstPtr& clock)
{
	if (flagTime)
	{
		startTime = getCurrentTime(clock);
		flagTime = false;
	}
}

void SnakeMv::clockCallback(const rosgraph_msgs::Clock::ConstPtr& clock)
{
	// if (isRotation)
	// {
	// 	initStartTime(clock);
	// 	if (getCurrentTime(clock) - startTime >= ANGLE_ROTATION / SPEED)
	// 	{
	// 		ROS_INFO("clockCallback isRotation");
	// 		stop();
	// 		if (numRotation % 2 == 0)
	// 			isAfterOddRotation = true;
	// 		numRotation = (numRotation + 1) % 4;
	// 		isRotation = false;
	// 		flagTime = true;
	// 	}
	// }
	// else if (isAfterOddRotation)
	// {
	// 	ROS_INFO("99999999999999999999999999999999999999999999");
	// 	initStartTime(clock);
	// 	if (getCurrentTime(clock) - startTime >= DIST_SHORT_SIDE / SPEED)
	// 	{
	// 		ROS_INFO("clockCallback isAfterOddRotation");
	// 		stop();
	// 		isRotation = true;
	// 		isAfterOddRotation = false;
	// 		flagTime = true;
	// 	}
	// }
}

void SnakeMv::startMoving()
{
    // ros::Rate rate(10);
	// isRotation = true;
    ros::Rate rate(1000);
    ROS_INFO("Start moving");
    while (ros::ok()) {
		if (!isObstacle && !isRotation)
        	moveLinear();
		else if (isRotation)
		{
			ROS_INFO("numRotation = %d", numRotation);
			if (numRotation < 2)
				moveAngular(false);
			else
				moveAngular(true);
		}
		// moveAngular(false);
        ros::spinOnce();
        rate.sleep();
    }
}
