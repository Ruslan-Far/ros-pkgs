#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"

ros::ServiceClient client;
nav_msgs::OccupancyGrid cleanedMap;

// void duplicateCleanedMap(nav_msgs::OccupancyGrid& cleanedMapFromSrv)
// {
	// cleanedMap.header.seq = cleanedMapFromSrv.header.seq;
	// cleanedMap.header.stamp = cleanedMapFromSrv.header.stamp;
	// cleanedMap.header.frame_id = cleanedMapFromSrv.header.frame_id;

	// cleanedMap.info.map_load_time = cleanedMapFromSrv.info.map_load_time;
	// cleanedMap.info.resolution = cleanedMapFromSrv.info.resolution;
	// cleanedMap.info.width = cleanedMapFromSrv.info.width;
	// cleanedMap.info.height = cleanedMapFromSrv.info.height;
	// cleanedMap.info.origin.position.x = cleanedMapFromSrv.info.origin.position.x;
	// cleanedMap.info.origin.position.y = cleanedMapFromSrv.info.origin.position.y;
	// cleanedMap.info.origin.position.z = cleanedMapFromSrv.info.origin.position.z;
	// cleanedMap.info.origin.orientation.x = cleanedMapFromSrv.info.origin.orientation.x;
	// cleanedMap.info.origin.orientation.y = cleanedMapFromSrv.info.origin.orientation.y;
	// cleanedMap.info.origin.orientation.z = cleanedMapFromSrv.info.origin.orientation.z;
	// cleanedMap.info.origin.orientation.w = cleanedMapFromSrv.info.origin.orientation.w;

	// cleanedMap.data = cleanedMapFromSrv.data;
// }

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
		// duplicateCleanedMap(srv.response.cleanedMap);
		cleanedMap = srv.response.cleanedMap;
		// ROS_INFO("cleanedMap.info.width = %d", cleanedMap.info.width);
		// ROS_INFO("srv.response.cleanedMap.info.width = %d", srv.response.cleanedMap.info.width);
		// srv.response.cleanedMap.info.width = 7;
		// ROS_INFO("changed cleanedMap.info.width = %d", cleanedMap.info.width);
		// ROS_INFO("changed srv.response.cleanedMap.info.width = %d", srv.response.cleanedMap.info.width);
		// ROS_INFO("cleanedMap.header.seq = %d", cleanedMap.header.seq);
		// ROS_INFO("srv.response.cleanedMap.header.seq = %d", srv.response.cleanedMap.header.seq);
		// srv.response.cleanedMap.header.seq = 7;
		// ROS_INFO("changed cleanedMap.header.seq = %d", cleanedMap.header.seq);
		// ROS_INFO("changed srv.response.cleanedMap.header.seq= %d", srv.response.cleanedMap.header.seq);
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
	ROS_INFO("iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");
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
		ROS_INFO("%d", cleanedMap.info.width);
		ROS_INFO("cleanedMap.header.seq = %d", cleanedMap.header.seq);

		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
