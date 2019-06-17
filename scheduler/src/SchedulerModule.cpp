#include "SchedulerModule.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

SchedulerModule::SchedulerModule(const int32_t &argc, char **argv) : 
	connected_modules(),
	frequency(1.0),
	timeout(100),
	scheduler_pub(),
	finished_module(""),
	scheduler_record() {
		
		const char *homedir = getenv("HOME");
		if(homedir == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}

		std::string path(homedir);

		path += "/sched_data.txt";
		
		scheduler_record.open(path, std::ofstream::out);

	}

SchedulerModule::~SchedulerModule() {}

void SchedulerModule::setUp() {
	//Search for active modules and connect them to scheduler?

	ros::NodeHandle param_handler("~"), scheduler_service_handler, scheduling_finish_handler;

	param_handler.getParam("frequency", frequency);
	int timeout_param;
	param_handler.getParam("timeout", timeout_param);

	timeout = static_cast<uint32_t>(timeout_param);

	scheduler_service = scheduler_service_handler.advertiseService("ModuleManagement", &SchedulerModule::moduleConnect, this);
	ros::spinOnce();

	schedule_finish = scheduling_finish_handler.subscribe("scheduling_finish", 1, &SchedulerModule::moduleFinishCallback, this);
}

void SchedulerModule::tearDown() {
	scheduler_record.close();
}

bool SchedulerModule::moduleConnect(services::SchedulerServerData::Request &req, services::SchedulerServerData::Response &res) {

	try {
		std::string topic_name = "";

		if(req.connection == true) {
			moduleParameters mp;
			mp.frequency = req.frequency;
			mp.relative_deadline = req.deadline;
			mp.wce = req.wce;
			mp.arrival_time = ros::Time::now();

			uint32_t sum_microsseconds = static_cast<uint32_t>(mp.arrival_time.nsec/1000UL) + mp.relative_deadline;
			uint32_t sum_seconds = mp.arrival_time.sec;

			while(sum_microsseconds >= 1000000UL) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time deadline(sum_seconds, sum_microsseconds*1000UL);
			mp.absolute_deadline = deadline;

			mp.executed_in_cycle = false;

			mp.task_counter.resize(2);
			mp.task_counter[0] = 0;
			mp.task_counter[1] = -1;

			topic_name = req.name.substr(1) + "topic";
			mp.topic_name = topic_name;

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );

				ros::Publisher pub = scheduler_topic_handler.advertise<std_msgs::String>(topic_name, 1);

				scheduler_pub[req.name] = pub;

				connected_modules[req.name] = mp;
			}

			std::cout << "Module Connected. Name: " << req.name << std::endl;
			std::cout << "Topic Name: " << topic_name << std::endl;
		} else {
			std::map<std::string,moduleParameters>::iterator modules_iterator;
			modules_iterator = connected_modules.find(req.name);
			
			std::map<std::string,ros::Publisher>::iterator publisher_iterator;
			publisher_iterator = scheduler_pub.find(req.name);

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );

				connected_modules.erase(modules_iterator);

				scheduler_pub.erase(publisher_iterator);
			}

			std::cout << "Module Disconnected. Name: " << req.name << std::endl;
		}

		res.ACK = true;
		res.topic_name = topic_name;

	} catch(...) {

		res.ACK = false;
		res.topic_name = "";

	}


	return true;

}

void SchedulerModule::moduleFinishCallback(const std_msgs::StringConstPtr& msg) {
	
	if(msg->data != "") {
		finished_module = msg->data;
	}
	
}

void SchedulerModule::run() {

	ros::Rate loop_rate(frequency);

	while(ros::ok()) {
		EDFSched(scheduler_pub);

		checkForDeadlineUpdate();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return tearDown();

}

std::set<std::pair<std::string, ros::Time>, Comparator> SchedulerModule::deadlinesSetCreation() {
	
	std::map<std::string, moduleParameters>::iterator modules_iterator;

    std::map<std::string, ros::Time> deadlines_map;

    {
    	std::unique_lock< std::mutex > lock( _modules_mutex );

	    for(modules_iterator = connected_modules.begin();modules_iterator != connected_modules.end();++modules_iterator) {

	        deadlines_map.insert(std::pair<std::string, ros::Time> (modules_iterator->first, modules_iterator->second.absolute_deadline));
	    
	    }
	}

    std::set<std::pair<std::string, ros::Time>, Comparator> set_of_deadlines(
        deadlines_map.begin(), deadlines_map.end(), comp
    );   

    return set_of_deadlines;

}

void SchedulerModule::EDFSched(std::map<std::string, ros::Publisher> &scheduler_pub) {
	
    std::set<std::pair<std::string, ros::Time>, Comparator> set_of_deadlines = deadlinesSetCreation();

    //Iterating through modules list in crescent order of deadlines
	for(std::set<std::pair<std::string, ros::Time>, Comparator>::iterator deadlines_it = set_of_deadlines.begin(); deadlines_it != set_of_deadlines.end(); ++deadlines_it) {

        std::map<std::string, moduleParameters>::iterator modules_iterator;

        {
        	std::unique_lock< std::mutex > lock( _modules_mutex );
        	
        	modules_iterator = connected_modules.find(std::get<0>(*(deadlines_it)));
        }

        if(modules_iterator != connected_modules.end()) {

        	bool invalid = false;

        	uint32_t sum_microsseconds = static_cast<uint32_t>(ros::Time::now().nsec/1000UL) + modules_iterator->second.wce;
			uint32_t sum_seconds = ros::Time::now().sec;

			while(sum_microsseconds >= 1000000UL && ros::ok()) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time time_limit(sum_seconds, sum_microsseconds*1000UL);

            try {
                //If we have time to execute, process job
                if(std::get<1>(*(deadlines_it)) >= time_limit) {
                                
                    std::string name = modules_iterator->first;
           
                    //If the module is able to execute, choose it
                    if(modules_iterator->second.executed_in_cycle == false) {
                                                
                    	std_msgs::String chosen_module;
                    	chosen_module.data = name;
                    	{
                    		std::unique_lock< std::mutex > lock( _modules_mutex );

                    		std::map<std::string, ros::Publisher>::iterator publisher_iterator;
                    		publisher_iterator = scheduler_pub.find(name);

                    		if(publisher_iterator != scheduler_pub.end()) {
		                    	scheduler_pub[name].publish(chosen_module);
		                    } else {
		                    	std::cout << std::endl;
		                    	std::cout << "Module " << name << " is disconnected! Scheduling next module." << std::endl;
		                    	std::cout << std::endl;
		                    	invalid = true;
		                    }
		                }

		                if(!invalid) {
		                    	scheduler_record << "Module Scheduled: " << name << std::endl;
		                    	scheduler_record << "Deadline: " << boost::posix_time::to_iso_extended_string(connected_modules[name].absolute_deadline.toBoost()) << std::endl;
		                    	
		                    	//Implement mechanism that waits for module to execute or timeout here
		                    	ros::Time init = ros::Time::now();
		                    	scheduler_record << "Initial Time: " << boost::posix_time::to_iso_extended_string(init.toBoost()) << std::endl;
		                    	ros::Duration timeout_time(0, timeout*1000000UL);
		                    	ros::Duration diff = ros::Time::now() - init;
		                    	
		                    	sum_microsseconds = static_cast<uint32_t>(connected_modules[name].absolute_deadline.nsec/1000UL) + connected_modules[name].wce;
		                    	sum_seconds = connected_modules[name].absolute_deadline.sec;

		                    	while(sum_microsseconds >= 1000000UL && ros::ok()) {
									sum_seconds++;
									sum_microsseconds -= 1000000UL;
								}

		                    	ros::Time max_time(sum_seconds, sum_microsseconds*1000UL);

		                    	while(diff < timeout_time && finished_module != name && ros::ok() && ros::Time::now() <= max_time) {

		                    		diff = ros::Time::now() - init;

		                    		ros::spinOnce();

		                    	}

		                    	chosen_module.data = "";
		                    	if(finished_module != name) { //Check if ROS node really consumes queue, if it does uncomment if
		                  			scheduler_record << "Finish Time: None" << std::endl << std::endl;
		                  			{
			                  			std::unique_lock< std::mutex > lock( _modules_mutex );

	                    				std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	                    				publisher_iterator = scheduler_pub.find(name);

	                    				if(publisher_iterator != scheduler_pub.end()) {
			                    			scheduler_pub[name].publish(chosen_module);
			                    		} else {
			                    			std::cout << std::endl;
			                    			std::cout << "Module " << name << " is disconnected! Scheduling next module." << std::endl;
			                    			std::cout << std::endl;
			                    			invalid = true;
			                    		}
			                    	}
			                    	if(!invalid) {
			                    		ros::spinOnce();
			                    	} else {
			                    		continue;
			                    	}
		                    	} else {
		                    		scheduler_record << "Finish Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl << std::endl;
		                    	}

		                    	finished_module = "";

		                    	modules_iterator = connected_modules.find(name);

		                    	if(modules_iterator != connected_modules.end()) {
		                    		modules_iterator->second.executed_in_cycle = true;
		                    	}
                        
                        }                         
                    
                    }

                }

            } catch(const std::exception& e) {
                std::clog << e.what() << std::endl;
            }

        }

    }

}

void SchedulerModule::checkForDeadlineUpdate() {

	//ROS_INFO("Checking for updates...");
	std::map<std::string, moduleParameters>::iterator modules_iterator;

	{
		std::unique_lock< std::mutex > lock( _modules_mutex );
	    for(modules_iterator = connected_modules.begin();modules_iterator != connected_modules.end();++modules_iterator) {

	        uint32_t sum_microsseconds = static_cast<uint32_t>(modules_iterator->second.absolute_deadline.nsec/1000UL) - modules_iterator->second.relative_deadline + static_cast<uint32_t>((1/modules_iterator->second.frequency)*1000000UL)*(modules_iterator->second.task_counter[0] - modules_iterator->second.task_counter[1]);
			uint32_t sum_seconds = modules_iterator->second.absolute_deadline.sec;

			while(sum_microsseconds >= 1000000UL) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time module_next_arrival(sum_seconds, sum_microsseconds*1000UL);

			if(ros::Time::now() >= module_next_arrival) {

				updateParameters(modules_iterator, module_next_arrival);

			}	
		
		}
	}

}

void SchedulerModule::updateParameters(std::map<std::string, moduleParameters>::iterator modules_iterator, ros::Time module_next_arrival) {

        ros::Duration difference;

        difference = ros::Time::now() - module_next_arrival;

        int aux;

        aux = static_cast<int>((difference.sec*1000000UL + difference.nsec/1000UL)/((1/modules_iterator->second.frequency)*1000000UL));

        aux += 1;

        modules_iterator->second.executed_in_cycle = false;
        modules_iterator->second.task_counter[1] = modules_iterator->second.task_counter[0];
        modules_iterator->second.task_counter[0] += aux;

		uint32_t sum_seconds = modules_iterator->second.absolute_deadline.sec;
        uint32_t d;

        d = static_cast<uint32_t>((modules_iterator->second.task_counter[0] - modules_iterator->second.task_counter[1])*((1/modules_iterator->second.frequency)*1000000UL));

        uint32_t sum_microsseconds = static_cast<uint32_t>(modules_iterator->second.absolute_deadline.nsec/1000UL) + d;

        while(sum_microsseconds >= 1000000UL) {
			sum_seconds++;
			sum_microsseconds -= 1000000UL;
		}
            
        modules_iterator->second.task_counter[1] = modules_iterator->second.task_counter[0] - 1;

        //Assigning new deadline value
        ros::Time new_deadline(sum_seconds, sum_microsseconds*1000UL);

        modules_iterator->second.absolute_deadline = new_deadline;

}