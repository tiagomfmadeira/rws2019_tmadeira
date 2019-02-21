#include <iostream>

int main(int argc, char* argv[])
{
	// std::cout << "Hello, World!" << std::endl;
	ros::init(argc, argv, "player_tmadeira");

	ros::NodeHandle n;

    for (int i = 0; i < 10; ++i)
    {
        std::count << i << std::endl;
    }

	return 0;
}