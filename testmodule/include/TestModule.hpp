#ifndef TEST_MODULE_HPP
#define TEST_MODULE_HPP
#include <string>
#include <vector>
#include <signal.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "std_msgs/String.h"
#include "ros/ros.h"

#include "services/SchedulerServerData.h"
#include "messages/FinishMessage.h"

struct moduleDescriptor {
	std::string name;
	float frequency;
	uint32_t deadline; //Microsseconds
	uint32_t wce; //Microsseconds
	bool connection;
	int priority;
};


class TestModule {

	private:
		TestModule(const TestModule &);
		TestModule &operator=(const TestModule &);

		void schedulingCallback(const std_msgs::StringConstPtr& msg);
	public:

		virtual void setUp();

		virtual void run();

		TestModule(const int32_t &argc, char **argv);

		virtual ~TestModule();

		static void schdulingSigIntHandler(int sig);

	private:
		moduleDescriptor md;

		std::string received_name;

		ros::Publisher scheduling_pub;

		ros::Subscriber sched;

		ros::ServiceClient client_module;

		std::string topic_name;

		std::string finish_topic_name;

};

#endif