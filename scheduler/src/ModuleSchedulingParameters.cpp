#include "ModuleSchedulingParameters.hpp"

ModuleSchedulingParameters::ModuleSchedulingParameters(const ros::Time &init, const ros::Time &max_time, const ros::Duration &diff, const ros::Time &finish_time) : 
	init(init),
	max_time(max_time),
	diff(diff),
	finish_time(finish_time) {}

ModuleSchedulingParameters::ModuleSchedulingParameters(const ModuleSchedulingParameters &obj) :
    init(obj.getInit()),
	max_time(obj.getMaxTime()),
	diff(obj.getDiff()),
	finish_time(obj.getFinishTime()) {}

ModuleSchedulingParameters& ModuleSchedulingParameters::operator=(const ModuleSchedulingParameters &obj) {
    this->init = obj.getInit();
    this->max_time = obj.getMaxTime();
    this->diff = obj.getDiff();
    this->finish_time = obj.getFinishTime();

    return (*this);
}

ModuleSchedulingParameters::ModuleSchedulingParameters() : init(), max_time(), diff(), finish_time() {}

ModuleSchedulingParameters::~ModuleSchedulingParameters() {}

void ModuleSchedulingParameters::setInit(const ros::Time &init){
    this->init = init;
}

ros::Time ModuleSchedulingParameters::getInit() const {
    return this->init;
}

void ModuleSchedulingParameters::setMaxTime(const ros::Time &max_time) {
    this->max_time = max_time;
}

ros::Time ModuleSchedulingParameters::getMaxTime() const {
    return this->max_time;
}

void ModuleSchedulingParameters::setDiff(const ros::Duration &diff) {
    this->diff = diff;
}

ros::Duration ModuleSchedulingParameters::getDiff() const {
    return this->diff;
}

void ModuleSchedulingParameters::setFinishTime(const ros::Time &finish_time) {
    this->finish_time = finish_time;
}

ros::Time ModuleSchedulingParameters::getFinishTime() const {
    return this->finish_time;
}