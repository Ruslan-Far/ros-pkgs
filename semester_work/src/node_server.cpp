#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "semester_work/CleanedMap.h"
#include <vector>

using namespace std;

vector<vector<bool>> grid;

void convertToGrid(int rows, int cols, signed char *data)
{
	int currCell;

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
            if (data[currCell] == 0)
                grid[i][j] = false;
            else
                grid[i][j] = true;
            currCell++;
        }
    }
}

void convertToArray(int rows, int cols, signed char *cleanedData)
{
	int currCell;

	currCell = 0;
    for (int i = 0; i < rows; i++)
	{
        for(int j = 0; j < cols; j++)
		{
			cleanedData[currCell] = (grid[i][j] ? 1 : 0);
            currCell++;
        }
    }
}

bool cleanMap(semester_work::CleanedMap::Request &req, semester_work::CleanedMap::Response &res)
{
	convertToGrid(req.height, req.width, req.data);
	convertToArray(req.height, req.width, res.cleanedData);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_server");
	ros::NodeHandle nh;
	ros::ServiceServer server = nh.advertiseService("cleaned_map", cleanMap);

	ROS_INFO("Ready to clean map");
	ros::spin();
	return 0;
}
