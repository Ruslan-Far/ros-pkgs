#include "SnakeMv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snake_mv");

    SnakeMv snakeMv;

    snakeMv.startMoving();

    return 0;
}
