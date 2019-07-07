#ifndef MODULESCHEDULINGPARAMETERS_H
#define MODULESCHEDULINGPARAMETERS_H

#include "ros/ros.h"

class ModuleSchedulingParameters {

	public:
		ModuleSchedulingParameters(const ros::Time &init, const ros::Time &max_time, const ros::Duration &diff, const ros::Time &finish_time);
		ModuleSchedulingParameters(const ModuleSchedulingParameters &);
		ModuleSchedulingParameters &operator=(const ModuleSchedulingParameters &);

		ModuleSchedulingParameters();
		~ModuleSchedulingParameters();

	public:
		void setInit(const ros::Time &init);
		ros::Time getInit() const;

		void setMaxTime(const ros::Time &max_time);
		ros::Time getMaxTime() const;

		void setDiff(const ros::Duration &diff);
		ros::Duration getDiff() const;

		void setFinishTime(const ros::Time &finish_time);
		ros::Time getFinishTime() const;

	private:
		ros::Time init;
		ros::Time max_time;
		ros::Duration diff;
		ros::Time finish_time;
};

#endif