#include "ModuleTemplate.hpp"

int main(int32_t argc, char **argv) {

	ros::init(argc, argv, "moduletemplate");
	
	ModuleTemplate module(argc, argv);

	module.setUp();
	module.run();

	return 0;

}