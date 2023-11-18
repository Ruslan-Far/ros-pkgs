#ifndef SNAKEMV_H
#define SNAKEMV_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <cmath>

class SnakeMv {
public:
  static constexpr double LINEAR_SPEED = 0.1;
  static constexpr double ANGULAR_SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI + (-40.0 / 180 * M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI + (40.0 / 180 * M_PI);
  static constexpr double ANGS_ROT[2]{-0.7, 0.001};
  static constexpr double AUX_ANGS_ROT[1]{-0.001};
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f;
  static constexpr float DIST_SHORT_SIDE = 0.5f;
  
  SnakeMv();
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
  int numRotation;
  double startPositionX;
  double startPositionY;

  void moveLinear();
  void moveAngular(bool direction);
  void stop();
  void initStartPositionXY(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void updateRotations();
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif