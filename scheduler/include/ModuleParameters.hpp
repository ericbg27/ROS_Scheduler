#ifndef MODULEPARAMETERS_HPP
#define MODULEPARAMETERS_HPP

#include <string>
#include <vector>

#include "ros/ros.h"

class ModuleParameters {

	public:
		ModuleParameters(const double &frequency, const uint32_t &relative_deadline, const uint32_t &wce, const ros::Time &arrival_time, const ros::Time &absolute_deadline, const std::vector<int> &task_counter, const std::string &topic_name, const std::string &finish_topic_name, const bool &active, const bool &executed_in_cycle, const int &priority);
		ModuleParameters(const ModuleParameters &);
		ModuleParameters &operator=(const ModuleParameters &);

		ModuleParameters();
		~ModuleParameters();

	public:
		void setFrequency(const double &frequency);
		double getFrequency() const;

		void setRelativeDeadline(const uint32_t &relative_deadline);
		uint32_t getRelativeDeadline() const;

		void setWorstCaseExecutionTime(const uint32_t &wce);
		uint32_t getWorstCaseExecutionTime() const;

		void setArrivalTime(const ros::Time &arrival_time);
		ros::Time getArrivalTime() const;

		void setAbsoluteDeadline(const ros::Time &absolute_deadline);
		ros::Time getAbsoluteDeadline() const;

		void setTaskCounter(const int &value, const int &position);
		std::vector<int>  getTaskCounter() const;

		void setTopicName(const std::string  &topic_name);
		std::string  getTopicName() const;

		void setFinishTopicName(const std::string  &finish_topic_name);
		std::string  getFinishTopicName() const;

		void setActive(const bool  &active);
		bool isActive() const;

		void setExecutedInCycle(const bool  &executed_in_cycle);
		bool isExecutedInCycle() const;

		void setPriority(const int  &priority);
		int getPriority() const;

	private:
		double frequency;
		uint32_t relative_deadline; //Microsseconds
		uint32_t wce; //Microsseconds
		ros::Time arrival_time; 
		ros::Time absolute_deadline;
		std::vector<int> task_counter;
		std::string topic_name;
		std::string finish_topic_name;
		bool active;
		bool executed_in_cycle;
		int priority;
};

#endif