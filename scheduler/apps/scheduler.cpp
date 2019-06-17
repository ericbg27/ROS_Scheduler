#include "SchedulerModule.hpp"

int main(int32_t argc, char **argv) {

	ros::init(argc, argv, "scheduler");
	
	SchedulerModule scheduler(argc, argv);

	scheduler.setUp();
	scheduler.run();

	return 0;

}
