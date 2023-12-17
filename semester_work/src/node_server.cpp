#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedGrid.h"

bool cleanGrid(semester_work::CleanedGrid::Request &req, semester_work::CleanedGrid::Response &res)
{
	res.cleanedGrid = req.grid;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_server");
	ros::NodeHandle nh;

	ros::ServiceServer server = nh.advertiseService("cleaned_grid", cleanGrid);
	ROS_INFO("Ready to clean grid");
	ros::spin();

	return 0;
}
