#include <iostream>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "player_tmadeira");

	ros::NodeHandle n;


    for (int i = 0; i < 10; ++i)
    {
        std::cout << i << std::endl;
    }

	return 0;
}