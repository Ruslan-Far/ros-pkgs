#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"
// #include <fstream>

ros::NodeHandle nh;
nav_msgs::OccupancyGrid cleanedMap;

void duplicateCleanedMap(nav_msgs::OccupancyGrid& cleanedMapFromSrv)
{
	cleanedMap.header.seq = cleanedMapFromSrv.header.seq;
	cleanedMap.header.stamp = cleanedMapFromSrv.header.stamp;
	cleanedMap.header.frame_id = cleanedMapFromSrv.header.frame_id;

	cleanedMap.info.map_load_time = cleanedMapFromSrv.info.map_load_time;
	cleanedMap.info.resolution = cleanedMapFromSrv.info.resolution;
	cleanedMap.info.width = cleanedMapFromSrv.info.width;
	cleanedMap.info.height = cleanedMapFromSrv.info.height;
	cleanedMap.info.origin.position.x = cleanedMapFromSrv.info.origin.position.x;
	cleanedMap.info.origin.position.y = cleanedMapFromSrv.info.origin.position.y;
	cleanedMap.info.origin.position.z = cleanedMapFromSrv.info.origin.position.z;
	cleanedMap.info.origin.orientation.x = cleanedMapFromSrv.info.origin.orientation.x;
	cleanedMap.info.origin.orientation.y = cleanedMapFromSrv.info.origin.orientation.y;
	cleanedMap.info.origin.orientation.z = cleanedMapFromSrv.info.origin.orientation.z;
	cleanedMap.info.origin.orientation.w = cleanedMapFromSrv.info.origin.orientation.w;

	cleanedMap.data.resize(cleanedMapFromSrv.data.size());
	for (int i = 0; i < cleanedMap.data.size(); i++)
		cleanedMap.data[i] = cleanedMapFromSrv.data[i];
}

void mapCallback(const nav_msgs::OccupancyGrid& map)
{
	ros::ServiceClient client = nh.serviceClient<semester_work::CleanedMap>("cleaned_map");
	semester_work::CleanedMap srv;

	while (!ros::service::waitForService("cleaned_map", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service cleaned_map to become available");
    }
	srv.request.map = map;
	if (client.call(srv))
	{
		duplicateCleanedMap(srv.response.cleanedMap);
		// cleanedMap = srv.response.cleanedMap;
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_client");
	ros::Publisher newMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/new_map", 1);
	ros::Subscriber mapSub = nh.subscribe("/map", 1, mapCallback);
	ros::Rate rate(10);

    ROS_INFO("Start");
	while (ros::ok())
	{
		newMapPub.publish(cleanedMap);
		ros::spinOnce();
		rate.sleep();
	}
    // if (!requestMap(nh))
    //     exit(-1);
    // printGridToFile();
    return 0;
}






// void readMap(const nav_msgs::OccupancyGrid& map)
// {
//     ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width, map.info.height, map.info.resolution);

//     rows = map.info.height;
//     cols = map.info.width;
//     mapResolution = map.info.resolution;

//     grid.resize(rows); // Dynamically resize the grid
//     for (int i=0; i<rows; i++) { grid[i].resize(cols); }

//     int currCell = 0;
//     for (int i=0; i<rows; i++) {
//         for(int j=0; j<cols; j++) {
//             if (map.data[currCell] == 0) // unoccupied cell
//                 grid[i][j] = false;
//             else
//                 grid[i][j] = true; //occupied (100) or unknown cell (-1)
//             currCell++;
//         }
//     }
// }

// bool requestMap(ros::NodeHandle &nh)
// {
//     nav_msgs::GetMap::Request req;
//     nav_msgs::GetMap::Response res;

//     while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
//          ROS_INFO("Waiting for service static_map to become available");
//     }

//     ROS_INFO("Requesting the map...");
//     ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
 
//     if (mapClient.call(req, res)) {
//         readMap(res.map);
//         return true;
//     }
//     else {
//         ROS_ERROR("Failed to call map service");
//         return false;
//     }
// }

// void printGridToFile() {
//     ROS_INFO("Print info to file grid.txt");
//     std::ofstream gridFile;
//     gridFile.open("grid.txt");
  
//     for (int i = grid.size() - 1; i >= 0; i--) {        
//         for (int j = 0; j < grid[i].size(); j++) {
//         gridFile << (grid[i][j] ? "1" : "0");           
//         }
//         gridFile << endl;
//     }
//     gridFile.close();
// }