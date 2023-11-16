#ifndef SNAKEMV_H
# define SNAKEMV_H

# include "ros/ros.h"
# include "sensor_msgs/LaserScan.h"
# include "geometry_msgs/Twist.h"

class SnakeMv {
public:
  static constexpr double FORWARD_SPEED = 0.1;
  static constexpr double MIN_SCAN_ANGLE = M_PI + (-40.0 / 180 * M_PI);
  static constexpr double MAX_SCAN_ANGLE = M_PI + (40.0 / 180 * M_PI);
  static constexpr float MIN_DIST_FROM_OBSTACLE = 0.4f;
  static constexpr float LENGTH_SHORT_SIDE = 0.5f;
  SnakeMv();
  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub;
  ros::Subcriber commandSub;
  ros::Subscriber laserSub;
  bool isObstacle;
  int numRotate;
  bool isAfterOddRotate;

  void moveForward();
  void moveAngle(bool direction);
  void stop();
  void poseCallback(const turtlesim::PoseConstPtr& msg)
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif