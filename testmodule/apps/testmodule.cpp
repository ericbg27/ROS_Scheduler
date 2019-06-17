#include "TestModule.hpp"

int main(int32_t argc, char **argv) {

	TestModule module(argc, argv);

	ros::init(argc, argv, "testmodule", ros::init_options::NoSigintHandler);

	module.setUp();
	module.run();

	return 0;
}