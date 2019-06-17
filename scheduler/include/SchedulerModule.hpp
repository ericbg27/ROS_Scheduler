#ifndef SCHEDULER_MODULE_HPP
#define SCHEDULER_MODULE_HPP

#include <string>
#include <vector>
#include <mutex>

#include "std_msgs/String.h"

#include "ros/ros.h"
//#include <ros/callback_queue.h>

#include "services/SchedulerServerData.h"
#include "ModuleParameters.h"

typedef std::function<bool(std::pair<std::string, ros::Time>, std::pair<std::string, ros::Time>)> Comparator;

const Comparator comp = [](std::pair<std::string, ros::Time> D1, std::pair<std::string, ros::Time> D2) {
    return D1.second < D2.second;
};

class SchedulerModule {

	private:
		SchedulerModule(const SchedulerModule &);
		SchedulerModule &operator=(const SchedulerModule &); //Talvez inútil

		//virtual void tearDown();

	public:
		virtual void setUp();

		bool moduleConnect(services::SchedulerServerData::Request &req, services::SchedulerServerData::Response &res);

		std::set<std::pair<std::string, ros::Time>, Comparator> deadlinesSetCreation();

		bool checkForEmptySchedulerCallbackQueue(ros::NodeHandle &scheduler_topic_handler);
		void emptySchedulerCallbackQueue(ros::NodeHandle &scheduler_topic);

		void EDFSched(std::map<std::string,ros::Publisher> &scheduler_pub);

		void checkForDeadlineUpdate();
		void updateParameters(std::map<std::string, moduleParameters>::iterator modules_iterator, ros::Time module_next_arrival);

		SchedulerModule(const int32_t &argc, char **argv);
		
		virtual ~SchedulerModule();

		void moduleFinishCallback(const std_msgs::StringConstPtr& msg);

		void run();

	private:
		std::map<std::string, moduleParameters> connected_modules;

		float frequency;

		uint32_t timeout; //timeout in milisseconds

		std::map<std::string,ros::Publisher> scheduler_pub;

		std::string finished_module;

		ros::ServiceServer scheduler_service;

		ros::Subscriber schedule_finish;

		std::mutex _modules_mutex;

		ros::NodeHandle scheduler_topic_handler;
};

#endif
