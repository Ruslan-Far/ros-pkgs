#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"
// #include <fstream>
#include <vector>

using namespace std;

ros::NodeHandle nh;
nav_msgs::OccupancyGrid cleanedMap;
bool isInitCleanedMap = false;

void duplicate(vector<signed char> dst, vector<signed char> src, int len)
{
	for (int i = 0; i < len; i++)
		dst[i] = src[i];
}

void fillCleanedMap(const nav_msgs::OccupancyGrid& map, signed char *cleanedData)
{
	int currCell;

	currCell = 0;
	cleanedMap.header = map.header;
	cleanedMap.info = map.info;
	if (!isInitCleanedMap)
		isInitCleanedMap = true;
	else
		delete [] cleanedMap.data;
	cleanedMap.data = new signed char[cleanedMap.info.width * cleanedMap.info.height];
	duplicate(cleanedMap.data, cleanedData, cleanedMap.info.width * cleanedMap.info.height);
}

void mapCallback(const nav_msgs::OccupancyGrid& map)
{
	ros::ServiceClient client = nh.serviceClient<semester_work::CleanedMap>("cleaned_map");
	semester_work::CleanedMap srv;

	while (!ros::service::waitForService("cleaned_map", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service cleaned_map to become available");
    }
	srv.request.width = map.info.width;
	srv.request.height = map.info.height;
	srv.request.data = new signed char[map.info.width * map.info.height];
	srv.response.cleanedData = new signed char[map.info.width * map.info.height];
	duplicate(srv.request.data, map.data, map.info.width * map.info.height);
	if (client.call(srv))
	{
		fillCleanedMap(map, srv.response.cleanedData);
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
	delete [] srv.request.data;
	delete [] srv.response.cleanedData;
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
	if (isInitCleanedMap)
		delete [] cleanedMap.data;
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