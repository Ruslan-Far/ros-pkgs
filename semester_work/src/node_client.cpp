#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedGrid.h"
#include <vector>
#include <fstream>

ros::NodeHandle nh;
vector<vector<bool>> grid;
nav_msgs::OccupancyGrid cleanedMap;

void readMap(const nav_msgs::OccupancyGrid& map)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width, map.info.height, map.info.resolution);

    rows = map.info.height;
    cols = map.info.width;
    mapResolution = map.info.resolution;

    grid.resize(rows); // Dynamically resize the grid
    for (int i=0; i<rows; i++) { grid[i].resize(cols); }

    int currCell = 0;
    for (int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            if (map.data[currCell] == 0) // unoccupied cell
                grid[i][j] = false;
            else
                grid[i][j] = true; //occupied (100) or unknown cell (-1)
            currCell++;
        }
    }
}

bool requestMap(ros::NodeHandle &nh)
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
         ROS_INFO("Waiting for service static_map to become available");
    }

    ROS_INFO("Requesting the map...");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
 
    if (mapClient.call(req, res)) {
        readMap(res.map);
        return true;
    }
    else {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

void printGridToFile() {
    ROS_INFO("Print info to file grid.txt");
    std::ofstream gridFile;
    gridFile.open("grid.txt");
  
    for (int i = grid.size() - 1; i >= 0; i--) {        
        for (int j = 0; j < grid[i].size(); j++) {
        gridFile << (grid[i][j] ? "1" : "0");           
        }
        gridFile << endl;
    }
    gridFile.close();
}

void convertToGrid(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	int rows;
	int cols;
	int currCell;

	rows = map.info.height;
	cols = map.info.width;
	currCell = 0;
    grid.resize(rows);
    for (int i = 0; i < rows; i++)
	{
		grid[i].resize(cols);
	}
    for (int i = 0; i < rows; i++)
	{
        for(int j = 0; j < cols; j++)
		{
            if (map.data[currCell] == 0)
                grid[i][j] = false;
            else
                grid[i][j] = true;
            currCell++;
        }
    }
}

void fillCleanedMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	int currCell;

	currCell = 0;
	cleanedMap.header = map.header;
	cleanedMap.info = map.info;
	cleanedMap.data[cleanedMap.info.height * cleanedMap.info.width];
    for (int i = 0; i < cleanedMap.info.height; i++)
	{
        for(int j = 0; j < cleanedMap.info.width; j++)
		{
			cleanedMap.data[currCell] = (grid[i][j] ? 1 : 0);
            currCell++;
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	convertToGrid(map);
	ros::ServiceClient client = nh.serviceClient<semester_work::CleanedGrid>("cleaned_grid");
	semester_work::CleanedGrid srv;
	srv.request.grid = grid;
	while (!ros::service::waitForService("cleaned_grid", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service cleaned_grid to become available");
    }
	if (client.call(srv))
	{
		grid = srv.response.cleanedGrid;
		fillCleanedMap(map);
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
