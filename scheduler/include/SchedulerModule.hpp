#ifndef SCHEDULER_MODULE_HPP
#define SCHEDULER_MODULE_HPP

#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <thread>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <condition_variable>

#include "std_msgs/String.h"

#include "ros/ros.h"
//#include <ros/callback_queue.h>

#include "services/SchedulerServerData.h"
#include "messages/Finish.h"
#include "messages/Init.h"
#include "ModuleParameters.hpp"
#include "ModuleSchedulingParameters.hpp"

struct Comparator
{
	bool operator()(std::pair<std::string, ros::Time> D1, std::pair<std::string, ros::Time> D2) {
		if(D1.first == D2.first || D1.first != D2.first) {
    		return D1.second < D2.second;
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

		void EDFSched(/*std::map<std::string,ros::Publisher> &scheduler_pub*/);

		void checkForDeadlineUpdate();
		void updateParameters(std::map<std::string, ModuleParameters>::iterator modules_iterator, ros::Time module_next_arrival);

		SchedulerModule(const int32_t &argc, char **argv);

		SchedulerModule();
		
		virtual ~SchedulerModule();

		void moduleFinishCallback(const messages::Finish::ConstPtr& msg);

		void run();

		void coordinateModules();

	private:
		std::map<std::string, ModuleParameters> connected_modules;

		std::map<std::string, ModuleSchedulingParameters> scheduling_modules;

		float frequency;

		uint32_t timeout; //timeout in milisseconds

		//std::map<std::string,ros::Publisher> scheduler_pub;
		ros::Publisher schedule_pub;

		std::vector<std::string> finished_modules;

		ros::ServiceServer scheduler_service;

		//std::map<std::string, ros::Subscriber> schedule_finish;
		ros::Subscriber finish_sub;

		std::mutex _modules_mutex, _ready_queue_sync;

		ros::NodeHandle scheduler_topic_handler, scheduling_finish_handler;

		std::ofstream scheduler_record;

		std::vector<std::string> scheduling_queue;

		//std::vector<std::string> ready_queue;

		ros::Duration timeout_time;

		std::condition_variable ready, deleting;

		bool sync, deleting_sync;

		std::set<std::pair<std::string, ros::Time>, Comparator> ready_queue;

		//std::thread thread1();

		//std::thread thread2();
};

#endif
