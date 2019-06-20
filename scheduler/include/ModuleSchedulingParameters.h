#ifndef MODULESCHEDULINGPARAMETERS_H
#define MODULESCHEDULINGPARAMETERS_H

#include "ros/ros.h"

struct moduleSchedulingParameters {
	ros::Time init;
	ros::Time max_time;
	ros::Duration diff;
	ros::Time finish_time;
};

#endif