#include "ModuleParameters.hpp"

ModuleParameters::ModuleParameters(const double &frequency, const uint32_t &relative_deadline, const uint32_t &wce, const ros::Time &arrival_time, const ros::Time &absolute_deadline, const std::vector<int> &task_counter, const std::string &topic_name, const std::string &finish_topic_name, const bool &active, const bool &executed_in_cycle) : 
	frequency(frequency),
	relative_deadline(relative_deadline),
	wce(wce),
	arrival_time(arrival_time),
	absolute_deadline(absolute_deadline),
	task_counter(task_counter),
    topic_name(topic_name),
    finish_topic_name(finish_topic_name),
    active(active),
    executed_in_cycle(executed_in_cycle) {}

ModuleParameters::ModuleParameters(const ModuleParameters &obj) :
    frequency(obj.getFrequency()),
	relative_deadline(obj.getRelativeDeadline()),
	wce(obj.getWorstCaseExecutionTime()),
	arrival_time(obj.getArrivalTime()),
	absolute_deadline(obj.getAbsoluteDeadline()),
	task_counter(obj.getTaskCounter()),
    topic_name(obj.getTopicName()),
    finish_topic_name(obj.getFinishTopicName()),
    active(obj.isActive()),
    executed_in_cycle(obj.isExecutedInCycle()) {}

ModuleParameters& ModuleParameters::operator=(const ModuleParameters &obj) {
    this->frequency = obj.getFrequency();
    this->relative_deadline = obj.getRelativeDeadline();
    this->wce = obj.getWorstCaseExecutionTime();
    this->arrival_time = obj.getArrivalTime();
    this->absolute_deadline = obj.getAbsoluteDeadline();
    this->topic_name = obj.getTopicName();
    this->finish_topic_name = obj.getFinishTopicName();
    this->active = obj.isActive();
    this->executed_in_cycle = obj.isExecutedInCycle();

    return (*this);
}

ModuleParameters::ModuleParameters() : 
	frequency(),
	relative_deadline(),
	wce(),
	arrival_time(),
	absolute_deadline(),
	task_counter(),
    topic_name(),
    finish_topic_name(),
    active(),
    executed_in_cycle() {}

ModuleParameters::~ModuleParameters() {}

const void ModuleParameters::setFrequency(const double &frequency){
    this->frequency = frequency;
}

double ModuleParameters::getFrequency() const {
    return this->frequency;
}

const void ModuleParameters::setRelativeDeadline(const uint32_t &relative_deadline) {
    this->relative_deadline = relative_deadline;
}

uint32_t ModuleParameters::getRelativeDeadline() const {
    return this->relative_deadline;
}

const void ModuleParameters::setWorstCaseExecutionTime(const uint32_t &wce) {
    this->wce = wce;
}

uint32_t ModuleParameters::getWorstCaseExecutionTime() const {
    return this->wce;
}

const void ModuleParameters::setArrivalTime(const ros::Time &arrival_time) {
    this->arrival_time = arrival_time;
}

ros::Time ModuleParameters::getArrivalTime() const {
    return this->arrival_time;
}

const void ModuleParameters::setAbsoluteDeadline(const ros::Time &absolute_deadline) {
    this->absolute_deadline = absolute_deadline;
}

ros::Time ModuleParameters::getAbsoluteDeadline() const {
    return this->absolute_deadline;
}

const void ModuleParameters::setTaskCounter(const int &value, const int &position) {
    this->task_counter[position] = value;
}

std::vector<int>  ModuleParameters::getTaskCounter() const {
    return this->task_counter;
}

const void ModuleParameters::setTopicName(const std::string  &topic_name) {
    this->topic_name = topic_name;
}

std::string  ModuleParameters::getTopicName() const {
    return this->topic_name;
}

const void ModuleParameters::setFinishTopicName(const std::string  &finish_topic_name) {
    this->finish_topic_name = finish_topic_name;
}

std::string  ModuleParameters::getFinishTopicName() const {
    return this->finish_topic_name;
}

const void ModuleParameters::setActive(const bool &active) {
    this->active = active;
}

bool ModuleParameters::isActive() const {
    return this->active;
}

const void ModuleParameters::setExecutedInCycle(const bool  &executed_in_cycle) {
    this->executed_in_cycle = executed_in_cycle;
}

bool ModuleParameters::isExecutedInCycle() const {
    return this->executed_in_cycle;
}