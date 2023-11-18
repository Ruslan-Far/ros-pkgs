#ifndef FREESPACE_H
#define FREESPACE_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <cmath>
#include <unistd.h>

class FreeSpace {
public:
  static constexpr double LINEAR_SPEED = 0.1;
  static constexpr double ANGULAR_SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI + (-20.0 / 180 * M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI + (20.0 / 180 * M_PI);
  static constexpr double ORIENT = 90.0;
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f;
  static constexpr float DIST_SHORT_SIDE = 0.5f;
  
  FreeSpace();
  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher cmdVelPub;
  ros::Subscriber modelStatesSub;
  ros::Subscriber scanSub;
  bool isObstacle;
  bool isRotation;
  bool isAfterOddRotation;
  bool flagPosition;
  bool flagOrient;
  int numRotation;
  double startPositionX;
  double startPositionY;
  double startOrient;

  void moveLinear();
  void moveAngular(bool direction);
  void stop();
  void initStartPositionXY(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void initStartOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  double getCurOrient(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
};

#endif
