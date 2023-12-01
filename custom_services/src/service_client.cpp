#include "ros/ros.h"
#include "custom_services/MeanThreeInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");
  if (argc != 4)
  {
    ROS_INFO("usage: simple_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<custom_services::MeanThreeInts>("mean_three_ints");
  custom_services::MeanThreeInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  srv.request.c = atoll(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Mean: %f", srv.response.mean);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
