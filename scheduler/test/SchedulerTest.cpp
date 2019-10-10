#include <gtest/gtest.h>
#include <vector>

#include "SchedulerModule.hpp"
#include "ros/ros.h"

using namespace std;

class SchedulerTest : public testing::Test {
	public:
		SchedulerTest() {};

		void setUp();

		void tearDown();

		SchedulerModule scheduler;

		services::SchedulerServerData::Request req;

		services::SchedulerServerData::Response res;
};

void SchedulerTest::setUp() {
	req.frequency = 1;
	req.deadline = 1000000;
	req.wce = 50;
	req.name = "/module";
	req.connection = true;
}

TEST_F (SchedulerTest, ModuleConnect) {
	scheduler.moduleConnect(req, res);

	ASSERT_EQ(true, res.ACK);
	ASSERT_EQ("moduletopic", res.topic_name);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
