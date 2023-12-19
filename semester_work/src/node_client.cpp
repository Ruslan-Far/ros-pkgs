#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"

ros::ServiceClient client;
nav_msgs::OccupancyGrid cleanedMap;

void mapCallback(const nav_msgs::OccupancyGrid& map)
{
	while (!ros::service::waitForService("cleaned_map", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service cleaned_map to become available");
    }
	semester_work::CleanedMap srv;
	srv.request.map = map;
	if (client.call(srv))
	{
		cleanedMap = srv.response.cleanedMap;
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_client");
	ros::NodeHandle nh;
	client = nh.serviceClient<semester_work::CleanedMap>("cleaned_map");
	ros::Publisher newMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/new_map", 1);
	ros::Subscriber mapSub = nh.subscribe("/map", 1, mapCallback);
	ros::Rate rate(10);

	while (ros::ok())
	{
		newMapPub.publish(cleanedMap);
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
