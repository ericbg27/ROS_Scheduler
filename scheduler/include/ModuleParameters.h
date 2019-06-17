#ifndef MODULEPARAMETERS_H
#define MODULEPARAMETERS_H

#include <string>
#include <vector>

struct moduleParameters {
	float frequency;
	uint32_t relative_deadline; //Microsseconds
	uint32_t wce; //Microsseconds
	ros::Time arrival_time; //Atualizar para std::chrono
	ros::Time absolute_deadline;
	bool executed_in_cycle;
	std::vector<int> task_counter;
	std::string topic_name;
};

#endif