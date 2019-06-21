#ifndef SCHEDULER_MODULE_HPP
#define SCHEDULER_MODULE_HPP

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <thread>
#include <tuple>
#include <iostream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <condition_variable>

#include "std_msgs/String.h"

#include "ros/ros.h"

#include "services/SchedulerServerData.h"
#include "messages/FinishMessage.h"
#include "ModuleParameters.h"
#include "ModuleSchedulingParameters.h"

struct Comparator
{
	bool operator()(std::tuple<std::string, ros::Time, int> D1, std::tuple<std::string, ros::Time, int> D2) {
		if(std::get<0>(D1) == std::get<0>(D2)|| std::get<0>(D1) != std::get<0>(D2)) {
			if(std::get<2>(D1) != std::get<2>(D2)) {
    			return std::get<2>(D1) < std::get<2>(D2);
    		} else {
    			return std::get<1>(D1) < std::get<1>(D1);
    		}
		}
	}
};

class SchedulerModule {

	private:
		SchedulerModule(const SchedulerModule &);
		SchedulerModule &operator=(const SchedulerModule &); //Talvez inútil

		virtual void tearDown();

	public:
		virtual void setUp();

		bool moduleConnect(services::SchedulerServerData::Request &req, services::SchedulerServerData::Response &res);

		std::set<std::pair<std::string, ros::Time>, Comparator> deadlinesSetCreation();

		void EDFSched();

		void checkForDeadlineUpdate();
		void updateParameters(std::map<std::string, moduleParameters>::iterator modules_iterator, ros::Time module_next_arrival);

		SchedulerModule(const int32_t &argc, char **argv);

		SchedulerModule();
		
		virtual ~SchedulerModule();

		void moduleFinishCallback(const messages::FinishMessage::ConstPtr& msg);
		//void moduleFinishCallback(const std_msgs::StringConstPtr& msg);

		void run();

		void coordinateModules();

		void Monitor();

		int PIDControl(std::vector<float> miss_ratio_vector, int actual_priority);

	private:
		std::map<std::string, moduleParameters> connected_modules;

		std::map<std::string, moduleSchedulingParameters> scheduling_modules;

		float frequency;

		float monitor_frequency;

		uint32_t timeout; //timeout in milisseconds

		std::map<std::string,ros::Publisher> scheduler_pub;

		std::vector<std::string> finished_modules;

		ros::ServiceServer scheduler_service;

		std::map<std::string, ros::Subscriber> schedule_finish;

		std::mutex _modules_mutex, _ready_queue_sync, _monitor_sync;

		ros::NodeHandle scheduler_topic_handler, scheduling_finish_handler;

		std::ofstream scheduler_record;

		std::vector<std::string> scheduling_queue;

		ros::Duration timeout_time;

		std::condition_variable ready, deleting, monitor;

		bool sync, deleting_sync, monitor_sync;

		//std::set<std::pair<std::string, ros::Time>, Comparator> ready_queue;

		std::set<std::tuple<std::string, ros::Time, int>, Comparator> ready_queue;

		//Controller variables
		int control_type;
		int DW, IW;
		float Kp, Kd, Ki;
};

#endif
