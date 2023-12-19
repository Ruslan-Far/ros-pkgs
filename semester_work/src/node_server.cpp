#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"
#include <vector>

using namespace std;

vector<vector<bool>> grid;

void convertToGrid(const nav_msgs::OccupancyGrid& map)
{
	int rows;
	int cols;
	int currCell;

	rows = map.info.height;
	cols = map.info.width;
	currCell = 0;
    grid.resize(rows);
	ROS_INFO("grid.size = %ld", grid.size());
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

void convertToArray(nav_msgs::OccupancyGrid& cleanedMap)
{
	int rows;
	int cols;
	int currCell;

	rows = cleanedMap.info.height;
	cols = cleanedMap.info.width;
	currCell = 0;
    for (int i = 0; i < rows; i++)
	{
        for(int j = 0; j < cols; j++)
		{
			cleanedMap.data[currCell] = (grid[i][j] ? 100 : 0);
            currCell++;
        }
    }
}

void fillCleanedMap(const nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cleanedMap)
{
	cleanedMap.header = map.header;
	cleanedMap.info = map.info;
	cleanedMap.data.resize(cleanedMap.info.width * cleanedMap.info.height);
	convertToArray(cleanedMap);
}

bool cleanMap(semester_work::CleanedMap::Request &req, semester_work::CleanedMap::Response &res)
{
	convertToGrid(req.map);
	// process
	fillCleanedMap(req.map, res.cleanedMap);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_server");
	ros::NodeHandle nh;
	ros::ServiceServer server = nh.advertiseService("cleaned_map", cleanMap);

	ROS_INFO("SERVER");
	// vector<int> v1;
	// vector<int> v2;

	// v1.resize(3);
	// v1[0] = 2;
	// v1[1] = 4;
	// v1[2] = 6;

	// for (int i = 0; i < v1.size(); i++)
	// 	ROS_INFO("%d", v1[i]);
	
	// v2.resize(3);
	// v2 = v1;

	// for (int i = 0; i < v2.size(); i++)
	// 	ROS_INFO("%d", v2[i]);

	// v2[1] = 9;
	// ROS_INFO("-----------------------------------------");
	// for (int i = 0; i < v1.size(); i++)
	// 	ROS_INFO("%d", v1[i]);
	// for (int i = 0; i < v2.size(); i++)
	// 	ROS_INFO("%d", v2[i]);

	ros::spin();
	return 0;
}
