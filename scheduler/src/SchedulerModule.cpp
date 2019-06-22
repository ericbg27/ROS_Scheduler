#include "SchedulerModule.hpp"

SchedulerModule::SchedulerModule(const int32_t &argc, char **argv) : 
	connected_modules(),
	frequency(5.0),
	monitor_frequency(0.5),
	timeout(500),
	scheduler_pub(),
	scheduler_record(),
	sync(true),
	deleting_sync(true),
	monitor_sync(true),
	control_type(0),
	IW(4),
	DW(2),
	Kp(-0.8),
	Kd(-4),
	Ki(-0.08) {
		
		const char *homedir = getenv("HOME");
		if(homedir == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}

		std::string path(homedir);

		path += "/sched_data.txt";
		
		scheduler_record.open(path, std::ofstream::out);

	}

SchedulerModule::SchedulerModule() :
	connected_modules(),
	frequency(5.0),
	monitor_frequency(0.5),
	timeout(500),
	scheduler_pub(),
	scheduler_record() {}
	
SchedulerModule::~SchedulerModule() {}

void SchedulerModule::setUp() {
	//Search for active modules and connect them to scheduler?

	ros::NodeHandle param_handler("~"), scheduler_service_handler;
	param_handler.getParam("frequency", frequency);
	param_handler.getParam("monitor_frequency", monitor_frequency);
	param_handler.getParam("control_type", control_type);
	int timeout_param;
	param_handler.getParam("timeout", timeout_param);

	timeout = static_cast<uint32_t>(timeout_param);

	ros::Duration timeout_time_aux(0, timeout*1000000UL);

	timeout_time = timeout_time_aux;

	scheduler_service = scheduler_service_handler.advertiseService("ModuleManagement", &SchedulerModule::moduleConnect, this);
	ros::spinOnce();
}

void SchedulerModule::tearDown() {
	scheduler_record.close();
}

bool SchedulerModule::moduleConnect(services::SchedulerServerData::Request &req, services::SchedulerServerData::Response &res) {

	try {
		std::string topic_name = "";
		std::string finish_topic_name = "";

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

			finish_topic_name = topic_name + "_finish";
			mp.finish_topic_name = finish_topic_name;

			mp.active = false;

			mp.jobs = 0;

			mp.failures = 0;

			int bigger_window;
			if(DW >= IW) {
				bigger_window = DW;
			} else if(IW > DW) {
				bigger_window = IW;
			}

			std::vector<float> empty_vector(bigger_window, 0.0);

			mp.miss_ratio_vector = empty_vector;

			mp.priority = req.priority;

			moduleSchedulingParameters mse;

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );
				deleting_sync = false;

				ros::Publisher pub = scheduler_topic_handler.advertise<std_msgs::String>(topic_name, 1);

				scheduler_pub[req.name] = pub;

				ros::Subscriber finish_sub = scheduling_finish_handler.subscribe(finish_topic_name, 1, &SchedulerModule::moduleFinishCallback, this);

				schedule_finish[req.name] = finish_sub;

				connected_modules[req.name] = mp;

				scheduling_modules[req.name] = mse;

				auto item = std::make_tuple(req.name, mp.absolute_deadline, mp.priority);
				ready_queue.insert(item);

				lock.unlock();
				deleting_sync = true;
				deleting.notify_all();
			}

			std::cout << "Module Connected. Name: " << req.name << std::endl;
			std::cout << "Priority: " << req.priority << std::endl;
			std::cout << "Scheduling Topic Name: " << topic_name << std::endl;
			std::cout << "Scheduling Finish Topic Name: " << finish_topic_name << std::endl;
		} else {
			std::map<std::string,moduleParameters>::iterator modules_iterator;
			modules_iterator = connected_modules.find(req.name);
			
			std::map<std::string,ros::Publisher>::iterator publisher_iterator;
			publisher_iterator = scheduler_pub.find(req.name);

			std::map<std::string,ros::Subscriber>::iterator subscriber_iterator;
			subscriber_iterator = schedule_finish.find(req.name);

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );
				deleting_sync = false;

				connected_modules.erase(modules_iterator);

				scheduler_pub.erase(publisher_iterator);

				schedule_finish.erase(subscriber_iterator);

				lock.unlock();
				deleting_sync = true;
				deleting.notify_all();
			}

			std::cout << "Module Disconnected. Name: " << req.name << std::endl;
		}

		res.ACK = true;
		res.topic_name = topic_name;
		res.finish_topic_name = finish_topic_name;

	} catch(...) {

		res.ACK = false;
		res.topic_name = "";
		res.finish_topic_name = "";

	}


	return true;

}

void SchedulerModule::moduleFinishCallback(const messages::FinishMessage::ConstPtr& msg) {
	
	if(msg->name != "") {
		if(connected_modules[msg->name].active == true) {
			ros::Time aux(msg->sec,msg->nsec);
			scheduler_record << "Finish scheduling module: " << msg->name << std::endl;
			scheduler_record << "Finish Time: " << boost::posix_time::to_iso_extended_string(aux.toBoost()) << std::endl;
			scheduler_record << "Received Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl << std::endl;
			scheduling_modules[msg->name].finish_time = aux;
			finished_modules.push_back(msg->name);
		}
	}
	
}

void SchedulerModule::run() {

	std::thread thread1(&SchedulerModule::EDFSched, this);
	std::thread thread2(&SchedulerModule::coordinateModules, this);
	std::thread thread3(&SchedulerModule::checkForDeadlineUpdate, this);
	std::thread thread4(&SchedulerModule::Monitor, this);

	thread1.join();
	thread2.join();
	thread3.join();
	thread4.join();
}

void SchedulerModule::EDFSched() {

	ros::Rate loop_rate(frequency);
	ros::spinOnce();

	while(ros::ok()) {
		{
			std::unique_lock< std::mutex > lk( _ready_queue_sync );
			ready.wait(lk, [this]{return sync;});
		}

		{
			std::unique_lock< std::mutex > lk( _ready_queue_sync );
			monitor.wait(lk, [this]{return monitor_sync;});
		}

		if(ready_queue.size() > 0) {

			//std::cout << "Getting ready queue element..." << std::endl;
			std::set<std::tuple<std::string, ros::Time, int>, Comparator>::iterator deadlines_it = ready_queue.begin();

	        std::map<std::string, moduleParameters>::iterator modules_iterator;

	        modules_iterator = connected_modules.find(std::get<0>(*(deadlines_it)));

	        ready_queue.erase(deadlines_it);
	        //std::cout << "Erase ready queue element..." << std::endl;
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
	                    if(modules_iterator->second.executed_in_cycle == false && modules_iterator->second.active == false) {
	                                                
	                    	scheduler_record << "Module Scheduled: " << name << std::endl;
			                scheduler_record << "Deadline: " << boost::posix_time::to_iso_extended_string(connected_modules[name].absolute_deadline.toBoost()) << std::endl;
	                    	//std::cout << "Chosen Module: " << chosen_module.data << std::endl;

	                    	{
	                    		std::unique_lock< std::mutex > lock( _modules_mutex );
	                    		deleting.wait(lock, [this]{return deleting_sync;});
	                    	}

	                    	std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	                    	publisher_iterator = scheduler_pub.find(name);

	                    	if(publisher_iterator != scheduler_pub.end()) {

	                    		std_msgs::String chosen_module;
	                    		chosen_module.data = name;

			                   	scheduler_pub[name].publish(chosen_module);
			                } else {
			                    std::cout << std::endl;
			                    std::cout << "Module " << name << " is disconnected! Scheduling next module." << std::endl;
			                    std::cout << std::endl;
			                    invalid = true;
			                }                

		                    if(!invalid) {
		                    	//std::cout << "Not invalid." << std::endl;
		                    	modules_iterator->second.active = true;

		                    	ros::Time init = ros::Time::now();
		                    	scheduler_record << "Initial Time: " << boost::posix_time::to_iso_extended_string(init.toBoost()) << std::endl << std::endl;
		                    	ros::Duration diff = ros::Time::now() - init;

		                    	sum_microsseconds = static_cast<uint32_t>(connected_modules[name].absolute_deadline.nsec/1000UL) - connected_modules[name].wce;
				                sum_seconds = connected_modules[name].absolute_deadline.sec;

				                while(sum_microsseconds >= 1000000UL && ros::ok()) {
									sum_seconds++;
									sum_microsseconds -= 1000000UL;
								}

				                ros::Time max_time(sum_seconds, sum_microsseconds*1000UL);

				                scheduling_modules[name].init = init;
				                scheduling_modules[name].diff = diff;
				                scheduling_modules[name].max_time = max_time;

				                //std::cout << "Pushing to queue" << std::endl;
		                    	scheduling_queue.push_back(name);
		                    }
		                }

	                }

	            } catch(const std::exception& e) {
	                std::clog << e.what() << std::endl;
	            }

	    	}
	    	}

	    ros::spinOnce();
	    loop_rate.sleep();
	}

	return tearDown();
}

void SchedulerModule::checkForDeadlineUpdate() {

	//ROS_INFO("Checking for updates...");
	ros::Rate loop_rate(frequency);

	while(ros::ok()) {
		{
            std::unique_lock< std::mutex > lock( _modules_mutex );
            deleting.wait(lock, [this]{return deleting_sync;});
        }

		std::map<std::string, moduleParameters>::iterator modules_iterator;

	    for(modules_iterator = connected_modules.begin();modules_iterator != connected_modules.end();++modules_iterator) {

	        uint32_t sum_microsseconds = static_cast<uint32_t>(modules_iterator->second.absolute_deadline.nsec/1000UL) - modules_iterator->second.relative_deadline + static_cast<uint32_t>((1/modules_iterator->second.frequency)*1000000UL)*(modules_iterator->second.task_counter[0] - modules_iterator->second.task_counter[1]);
			uint32_t sum_seconds = modules_iterator->second.absolute_deadline.sec;

			while(sum_microsseconds >= 1000000UL) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time module_next_arrival(sum_seconds, sum_microsseconds*1000UL);

			if(ros::Time::now() >= module_next_arrival && modules_iterator->second.active == false) {

				sync = false;

				updateParameters(modules_iterator, module_next_arrival);

			}	
		}

		//ros::spinOnce();
		loop_rate.sleep();
	}

}

void SchedulerModule::updateParameters(std::map<std::string, moduleParameters>::iterator modules_iterator, ros::Time module_next_arrival) {
		//std::cout << "updating Parameters" << std::endl;
		//std::cout << "Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl;
        ros::Duration difference;

        difference = ros::Time::now() - module_next_arrival;

        int aux;

        aux = static_cast<int>((difference.sec*1000000UL + difference.nsec/1000UL)/((1/modules_iterator->second.frequency)*1000000UL));

        aux += 1;


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

        //Assigning new deadline value
        ros::Time new_deadline(sum_seconds, sum_microsseconds*1000UL);

        {
        	std::unique_lock< std::mutex > lk( _ready_queue_sync );
        	modules_iterator->second.absolute_deadline = new_deadline;

        	if(!modules_iterator->second.executed_in_cycle) {
        		modules_iterator->second.failures += aux;
        	} else {
        		modules_iterator->second.failures += aux - 1;
        		//modules_iterator->second.failures += aux; //Erro proposital
        	}

        	modules_iterator->second.jobs += aux;

        	modules_iterator->second.executed_in_cycle = false;
        	modules_iterator->second.task_counter[1] = modules_iterator->second.task_counter[0] - 1;
        	//std::cout << "Inserting in ready_queue" << std::endl;
        	std::tuple<std::string, ros::Time, int> item = std::make_tuple(modules_iterator->first, new_deadline, modules_iterator->second.priority);

        	//std::set<std::tuple<std::string, ros::Time, int>, Comparator>::iterator ready_queue_iterator;

        	ready_queue.insert(item);
        	//std::cout << "Inserted new element" << std::endl;

        	sync = true;
        	ready.notify_all();

        	lk.unlock();
        }

}

void SchedulerModule::coordinateModules() {

	ros::Rate loop_rate(frequency);

	//ros::spinOnce();

	while(ros::ok()) {
	unsigned int i = 0;

	std::vector<std::string>::iterator finished_iterator;
	std::map<std::string, moduleParameters>::iterator modules_iterator;

	while( i < scheduling_queue.size() && ros::ok()) {
		//std::cout << "Entering coordinateModules loop" << std::endl;
		//std::cout << "Module name: " << scheduling_queue[i] << std::endl;

		/*messages::FinishMessage msg;
        msg.name = "";
        msg.sec = 0;
        msg.nsec = 0;*/
        std_msgs::String chosen_module;
        chosen_module.data = "";

        bool invalid = false;

        finished_iterator = std::find(finished_modules.begin(), finished_modules.end(), scheduling_queue[i]);
		if(finished_iterator != finished_modules.end()) {
			//std::cout << "Module Finished" << std::endl;
			//std::cout << "max_time: " << boost::posix_time::to_iso_extended_string(scheduling_modules[scheduling_queue[i]].max_time.toBoost()) << std::endl;
			//std::cout << "Module Name: " << scheduling_queue[i] << std::endl;

			if(scheduling_modules[scheduling_queue[i]].finish_time > scheduling_modules[scheduling_queue[i]].max_time) {
			    {
                    std::unique_lock< std::mutex > lock( _modules_mutex );
                    deleting.wait(lock, [this]{return deleting_sync;});
                }

	            std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	   			publisher_iterator = scheduler_pub.find(scheduling_queue[i]);

	            if(publisher_iterator != scheduler_pub.end()) {
			        scheduler_pub[scheduling_queue[i]].publish(chosen_module);
			    } else {
			        invalid = true;
			    }
			    //if(!invalid) {
			        //ros::spinOnce();
			    //}
			}

			modules_iterator = connected_modules.find(scheduling_queue[i]);

		    if(modules_iterator != connected_modules.end()) {
		        modules_iterator->second.executed_in_cycle = true;
		        modules_iterator->second.active = false;
		    }

		    finished_modules.erase(finished_iterator);
			scheduling_queue.erase(scheduling_queue.begin() + i);

		} else {

			scheduling_modules[scheduling_queue[i]].diff = ros::Time::now() - scheduling_modules[scheduling_queue[i]].init;

			if(scheduling_modules[scheduling_queue[i]].diff >= timeout_time || ros::Time::now() > scheduling_modules[scheduling_queue[i]].max_time) {
				scheduler_record << "Module Timeout" << std::endl;
				scheduler_record << "Module Name: " << scheduling_queue[i] << std::endl;
				scheduler_record << "Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl << std::endl;
				{
			        std::unique_lock< std::mutex > lock( _modules_mutex );

	                std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	   				publisher_iterator = scheduler_pub.find(scheduling_queue[i]);

	                if(publisher_iterator != scheduler_pub.end()) {
			            scheduler_pub[scheduling_queue[i]].publish(chosen_module);
			        } else {
			            invalid = true;
			        }
			    }

			    modules_iterator = connected_modules.find(scheduling_queue[i]);

		        if(modules_iterator != connected_modules.end()) {
		            modules_iterator->second.executed_in_cycle = true;
		            modules_iterator->second.active = false;
		        }

		        scheduling_queue.erase(scheduling_queue.begin() + i);
			} else {
				//std::cout << "Module's running" << std::endl;
				i++;
			}

		}
	}
	//ros::spinOnce();
	loop_rate.sleep();
	}
}

void SchedulerModule::Monitor() {
	ros::Rate loop_rate(monitor_frequency);

	while(ros::ok() && control_type == 1) {

		std::map<std::string, moduleParameters> aux;

		aux = connected_modules;

		std::map<std::string,moduleParameters>::iterator modules_iterator;

		std::map<std::string, std::vector<float>> miss_ratio_map;

		std::map<std::string, int> priority_map;

		for(modules_iterator = aux.begin();modules_iterator != aux.end();++modules_iterator) {

			//std::cout << "Module total jobs: " << modules_iterator->second.jobs << std::endl;
			//std::cout << "Module failures: " << modules_iterator->second.failures << std::endl;

			miss_ratio_map[modules_iterator->first] = modules_iterator->second.miss_ratio_vector;
			miss_ratio_map[modules_iterator->first].erase(miss_ratio_map[modules_iterator->first].begin());

			if(modules_iterator->second.jobs > 0) {
				miss_ratio_map[modules_iterator->first].push_back(100*modules_iterator->second.failures/modules_iterator->second.jobs);
			} else {
				miss_ratio_map[modules_iterator->first].push_back(0);
			}

			priority_map[modules_iterator->first] = PIDControl(miss_ratio_map[modules_iterator->first], modules_iterator->second.priority);
		}

		std::map<std::string,std::vector<float>>::iterator miss_ratio_iterator;

		{
			std::unique_lock< std::mutex > lk( _ready_queue_sync );
			ready.wait(lk, [this]{return sync;});
		}

		{
	       	std::unique_lock< std::mutex > lock( _modules_mutex );
	        deleting.wait(lock, [this]{return deleting_sync;});
	    }

		for(miss_ratio_iterator = miss_ratio_map.begin(); miss_ratio_iterator != miss_ratio_map.end(); ++miss_ratio_iterator) {

			if(connected_modules.find(miss_ratio_iterator->first) != connected_modules.end()) {
				connected_modules[miss_ratio_iterator->first].miss_ratio_vector = miss_ratio_iterator->second;
				connected_modules[miss_ratio_iterator->first].jobs = 0;
				connected_modules[miss_ratio_iterator->first].failures = 0;
			}

		}

		std::map<std::string, int>::iterator priority_iterator;

		{
			monitor_sync = false;
			std::unique_lock< std::mutex > mlk( _monitor_sync );

			for(priority_iterator = priority_map.begin();priority_iterator != priority_map.end(); ++priority_iterator) {
				if(connected_modules.find(priority_iterator->first) != connected_modules.end()) {
					connected_modules[priority_iterator->first].priority = priority_iterator->second;
					//std::cout << "Module: " << priority_iterator->first << " Priority: " << connected_modules[priority_iterator->first].priority << std::endl;
				}
			}

			monitor_sync = true;
        	monitor.notify_all();

        	mlk.unlock();	
		}

		/*for(modules_iterator = connected_modules.begin();modules_iterator != connected_modules.end();++connected_modules) {
			PIDControl(modules_iterator->second.miss_ratio, modules_iterator->second.priority);
		}*/

		loop_rate.sleep();
	}
}

int SchedulerModule::PIDControl(std::vector<float> miss_ratio_vector, int actual_priority) {

	int new_priority;
	float delta_priority;

	float sum = 0;

	int size = static_cast<int>(miss_ratio_vector.size());

	for(int i = 0;i < IW;i++) {
		sum += miss_ratio_vector[size-1-i];
	}

	//std::cout << "size-1 element: " << miss_ratio_vector[size-1] << std::endl;
	//std::cout << "size-1-DW element: " << miss_ratio_vector[size-DW-1] << std::endl;
	
	delta_priority = -Kp*miss_ratio_vector[size-1] - Ki*sum - Kd*((miss_ratio_vector[size-1] - miss_ratio_vector[size-DW-1])/DW);
	
	//std::cout << "delta_priority: " << delta_priority << std::endl;
	new_priority = actual_priority + static_cast<int>(round(delta_priority));

	if(new_priority < 0) {
		new_priority = 0;
	} else if(new_priority > 100) {
		new_priority = 100;
	}

	return new_priority;

}