#include "ros/ros.h"
#include "custom_services/MeanThreeInts.h"

bool mean(custom_services::MeanThreeInts::Request  &req,
          custom_services::MeanThreeInts::Response &res)
{
  res.mean = (req.a + req.b + req.c) / 3.0;
  ROS_INFO("request: x=%ld, y=%ld, z=%ld", (long int)req.a, (long int)req.b, (long int)req.c);
  ROS_INFO("sending back response: [%f]", res.mean);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("mean_three_ints", mean);
  ROS_INFO("Ready to calculate mean.");
  ros::spin();

  return 0;
}
