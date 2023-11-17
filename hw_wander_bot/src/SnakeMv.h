#ifndef SNAKEMV_H
#define SNAKEMV_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "rosgraph_msgs/Clock.h"
#include <cmath>

class SnakeMv {
public:
  static constexpr double SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI + (-40.0 / 180 * M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI + (40.0 / 180 * M_PI);
  static constexpr double ANGLE_ROTATION = M_PI / 2.0;
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f;
  static constexpr float DIST_SHORT_SIDE = 0.5f;
  SnakeMv();
  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher cmdVelPub;
  ros::Publisher setModelStatePub;
  ros::Subscriber modelStatesSub;
  ros::Subscriber scanSub;
  ros::Subscriber clockSub;
  bool isObstacle;
  bool isAfterOddRotation;
  bool flagTime;
  bool isRotation;
//   bool scanResolutionForMv;
  int numRotation;
  double startTime;

  void moveLinear();
  void moveAngular(bool direction);
  void stop();
  void setModelState();
  void setModelState2();
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void clockCallback(const rosgraph_msgs::Clock::ConstPtr& clock);
  double getCurrentTime(const rosgraph_msgs::Clock::ConstPtr& clock);
  void initStartTime(const rosgraph_msgs::Clock::ConstPtr& clock);
};

#endif