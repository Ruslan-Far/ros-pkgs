#include "FreeSpace.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "free_space");

    FreeSpace freeSpace;

    freeSpace.startMoving();

    return 0;
}
