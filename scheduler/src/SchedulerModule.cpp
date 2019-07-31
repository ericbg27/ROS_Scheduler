#include "SchedulerModule.hpp"

SchedulerModule::SchedulerModule(const int32_t &argc, char **argv) : 
	connected_modules(),
	scheduling_modules(),
	frequency(1),
	timeout(100),
	schedule_pub(),
	scheduler_record(),
	sync(true),
	deleting_sync(true) {
		
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
	scheduling_modules(),
	frequency(1),
	timeout(100),
	schedule_pub(),
	scheduler_record() {}
	
SchedulerModule::~SchedulerModule() {}

void SchedulerModule::setUp() {
	//Search for active modules and connect them to scheduler?

	ros::NodeHandle param_handler("~"), scheduler_service_handler;

	param_handler.getParam("frequency", frequency);
	int timeout_param;
	param_handler.getParam("timeout", timeout_param);

	timeout = static_cast<uint32_t>(timeout_param);

	ros::Duration timeout_time_aux(0, timeout*1000000UL);

	timeout_time = timeout_time_aux;

	scheduler_service = scheduler_service_handler.advertiseService("SchedulerRegister", &SchedulerModule::moduleConnect, this);
	schedule_pub = scheduler_topic_handler.advertise<messages::ReconfigurationCommand>("log_reconfigure", 1);
	finish_sub = scheduling_finish_handler.subscribe("event", 1, &SchedulerModule::receiveEvent, this);
	
	ros::spinOnce();
}

void SchedulerModule::tearDown() {
	scheduler_record.close();
}

bool SchedulerModule::moduleConnect(services::SchedulerRegister::Request &req, services::SchedulerRegister::Response &res) {

	try {
		//std::string topic_name = "";
		//std::string finish_topic_name = "";

		if(req.connection == true) {
			ros::Time arrival_time = ros::Time::now();

			uint32_t sum_microsseconds = static_cast<uint32_t>(arrival_time.nsec/1000UL) + req.deadline;
			uint32_t sum_seconds = arrival_time.sec;

			while(sum_microsseconds >= 1000000UL) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time abs_deadline(sum_seconds, sum_microsseconds*1000UL);

			ModuleParameters mp(req.frequency,
									req.deadline,
									req.wce,
									arrival_time,
									abs_deadline,
									{{0,-1}},
									req.name.substr(1) + "topic",
									req.name.substr(1) + "topic" + "_finish",
									false,
									false);

			ModuleSchedulingParameters mse;

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );
				deleting_sync = false;

				//ros::Publisher pub = scheduler_topic_handler.advertise<messages::ReconfigurationCommand>(mp.getTopicName(), 1);

				//schedule_pub[req.name] = pub;

				//ros::Subscriber finish_sub = scheduling_finish_handler.subscribe(mp.getFinishTopicName(), 1, &SchedulerModule::receiveEvent, this);

				//schedule_finish[req.name] = finish_sub;

				connected_modules[req.name] = mp;

				scheduling_modules[req.name] = mse;

				auto item = std::make_pair(req.name, mp.getAbsoluteDeadline());
				ready_queue.insert(item);

				lock.unlock();
				deleting_sync = true;
				deleting.notify_all();
			}

			std::cout << "Module Connected. Name: " << req.name << std::endl;
			//std::cout << "Scheduling Topic Name: " << mp.getTopicName() << std::endl;
			//std::cout << "Scheduling Finish Topic Name: " << mp.getFinishTopicName() << std::endl;
			std::cout << "Module Frequency: " << mp.getFrequency() << std::endl;
			//std::cout << "Module Task Counter 0: " << mp.getTaskCounter()[0] << std::endl;
			//std::cout << "Module Task Counter 1: " << mp.getTaskCounter()[1] << std::endl;
		} else {
			std::map<std::string,ModuleParameters>::iterator modules_iterator;
			modules_iterator = connected_modules.find(req.name);
			
			//std::map<std::string,ros::Publisher>::iterator publisher_iterator;
			//publisher_iterator = schedule_pub.find(req.name);

			//std::map<std::string,ros::Subscriber>::iterator subscriber_iterator;
			//subscriber_iterator = schedule_finish.find(req.name);

			{
				std::unique_lock< std::mutex > lock( _modules_mutex );
				deleting_sync = false;

				connected_modules.erase(modules_iterator);

				//schedule_pub.erase(publisher_iterator);

				//schedule_finish.erase(subscriber_iterator);

				lock.unlock();
				deleting_sync = true;
				deleting.notify_all();
			}

			std::cout << "Module Disconnected. Name: " << req.name << std::endl;
		}

		res.ACK = true;
//		res.topic_name = topic_name;
//		res.finish_topic_name = finish_topic_name;

	} catch(...) {

		res.ACK = false;
//		res.topic_name = "";
//		res.finish_topic_name = "";

	}


	return true;

}

void SchedulerModule::receiveEvent(const messages::Event::ConstPtr& msg) {
	
	if(msg->type != "finish") {
		if(connected_modules[msg->source].isActive()) {
			/* Oh no! Disctinct nodes distinct time references!!!!!!!!!!!!! (since we do not presume global clock)*/
			//ros::Time aux(msg->sec,msg->nsec);

			scheduler_record << "Finish scheduling module: " << msg->source << std::endl;
			scheduler_record << "Finish Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl;
			scheduler_record << "Received Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl << std::endl;

			scheduling_modules[msg->source].setFinishTime(ros::Time::now());
			finished_modules.push_back(msg->source);
		}
	}
	
}

void SchedulerModule::run() {

	std::thread thread1(&SchedulerModule::EDFSched, this);
	std::thread thread2(&SchedulerModule::coordinateModules, this);
	std::thread thread3(&SchedulerModule::checkForDeadlineUpdate, this);

	thread1.join();
	thread2.join();
	thread3.join();

}

void SchedulerModule::EDFSched() {

	ros::Rate loop_rate(frequency);
	//ros::spinOnce();

	while(ros::ok()) {
		ROS_INFO("[EDFSched] Scheduling...");

		{
			std::unique_lock< std::mutex > lk( _ready_queue_sync );
			ready.wait(lk, [this]{return sync;});
		}

		if(ready_queue.size() > 0) {

			ROS_INFO("Getting ready queue element...");
			std::set<std::pair<std::string, ros::Time>, Comparator>::iterator deadlines_it = ready_queue.begin();

	        std::map<std::string, ModuleParameters>::iterator modules_iterator;

	        modules_iterator = connected_modules.find(std::get<0>(*(deadlines_it)));

	        ready_queue.erase(deadlines_it);
	        ROS_INFO("Erase ready queue element...");
	        if(modules_iterator != connected_modules.end()) {

	        	bool invalid = false;

	        	uint32_t sum_microsseconds = static_cast<uint32_t>(ros::Time::now().nsec/1000UL) + modules_iterator->second.getWorstCaseExecutionTime();
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
	                    if(!modules_iterator->second.isExecutedInCycle() && !modules_iterator->second.isActive()) {
	                                                
	                    	scheduler_record << "Module Scheduled: " << name << std::endl;
			                scheduler_record << "Deadline: " << boost::posix_time::to_iso_extended_string(connected_modules[name].getAbsoluteDeadline().toBoost()) << std::endl;
	                    	//std::cout << "Chosen Module: " << reconfig.command << std::endl;

	                    	{
	                    		std::unique_lock< std::mutex > lock( _modules_mutex );
	                    		deleting.wait(lock, [this]{return deleting_sync;});
	                    	}

							/* old code
								std::map<std::string, ros::Publisher>::iterator publisher_iterator;
								publisher_iterator = schedule_pub.find(name);

								if(publisher_iterator != schedule_pub.end()) {
									messages::ReconfigurationCommand reconfig;
									reconfig.command = name;

									ROS_INFO("msg->source: [%s]", reconfig.command.c_str());
									schedule_pub[name].publish(reconfig);
								} else {
									std::cout << std::endl;
									std::cout << "Module " << name << " is disconnected! Scheduling next module." << std::endl;
									std::cout << std::endl;
									invalid = true;
								}

								if(!invalid) {
									//std::cout << "Not invalid." << std::endl;
									modules_iterator->second.setActive(true) ;

									ros::Time init = ros::Time::now();
									scheduler_record << "Initial Time: " << boost::posix_time::to_iso_extended_string(init.toBoost()) << std::endl << std::endl;
									ros::Duration diff = ros::Time::now() - init;

									sum_microsseconds = static_cast<uint32_t>(connected_modules[name].getAbsoluteDeadline().nsec/1000UL) - connected_modules[name].getWorstCaseExecutionTime();
									sum_seconds = connected_modules[name].getAbsoluteDeadline().sec;

									while(sum_microsseconds >= 1000000UL && ros::ok()) {
										sum_seconds++;
										sum_microsseconds -= 1000000UL;
									}

									ros::Time max_time(sum_seconds, sum_microsseconds*1000UL);

									scheduling_modules[name].setInit(init);
									scheduling_modules[name].setDiff(diff);
									scheduling_modules[name].setMaxTime(max_time);

									//std::cout << "Pushing to queue" << std::endl;
									scheduling_queue.push_back(name);
								}    
							*/
							std::map<std::string, ModuleParameters>::iterator it;
	                    	it = connected_modules.find(name);

	                    	if(it != connected_modules.end()) {
	                    		messages::ReconfigurationCommand reconfig;
	                    		reconfig.source = ros::this_node::getName();
								reconfig.target = name;
	                    		reconfig.action = "execute";

			                   	schedule_pub.publish(reconfig);

								modules_iterator->second.setActive(true) ;

		                    	ros::Time init = ros::Time::now();
		                    	scheduler_record << "Initial Time: " << boost::posix_time::to_iso_extended_string(init.toBoost()) << std::endl << std::endl;
		                    	ros::Duration diff = ros::Time::now() - init;

		                    	sum_microsseconds = static_cast<uint32_t>(connected_modules[name].getAbsoluteDeadline().nsec/1000UL) - connected_modules[name].getWorstCaseExecutionTime();
				                sum_seconds = connected_modules[name].getAbsoluteDeadline().sec;

				                while(sum_microsseconds >= 1000000UL && ros::ok()) {
									sum_seconds++;
									sum_microsseconds -= 1000000UL;
								}

				                ros::Time max_time(sum_seconds, sum_microsseconds*1000UL);

				                scheduling_modules[name].setInit(init);
				                scheduling_modules[name].setDiff(diff);
				                scheduling_modules[name].setMaxTime(max_time);

				                //std::cout << "Pushing to queue" << std::endl;
		                    	scheduling_queue.push_back(name);

			                } else {
			                    std::cout << std::endl;
			                    std::cout << "Module " << name << " is disconnected! Scheduling next module." << std::endl;
			                    std::cout << std::endl;
							}
		                }

	                }

	            } catch(const std::exception& e) {
	                std::clog << e.what() << std::endl;
	            }

	    	}
	    	}

	    ros::spinOnce();
	    ROS_INFO("[EDFSched] Finished Scheduling...");
	    loop_rate.sleep();
	}

	return tearDown();
}

void SchedulerModule::checkForDeadlineUpdate() {

	//ROS_INFO("Checking for updates...");
	ros::Rate loop_rate(frequency);

	while(ros::ok()) {
		//std::cout << "[checkForDeadlineUpdate] Checking for Updates..." << std::endl;
		{
            std::unique_lock< std::mutex > lock( _modules_mutex );
            deleting.wait(lock, [this]{return deleting_sync;});
        }
        //std::cout << "[checkForDeadlineUpdate] Executing Check..." << std::endl;

		std::map<std::string, ModuleParameters>::iterator modules_iterator;

	    for(modules_iterator = connected_modules.begin();modules_iterator != connected_modules.end();++modules_iterator) {

	        uint32_t sum_microsseconds = static_cast<uint32_t>(modules_iterator->second.getAbsoluteDeadline().nsec/1000UL) - modules_iterator->second.getRelativeDeadline() + static_cast<uint32_t>((1/modules_iterator->second.getFrequency())*1000000UL)*(modules_iterator->second.getTaskCounter()[0] - modules_iterator->second.getTaskCounter()[1]);
			uint32_t sum_seconds = modules_iterator->second.getAbsoluteDeadline().sec;

			while(sum_microsseconds >= 1000000UL) {
				sum_seconds++;
				sum_microsseconds -= 1000000UL;
			}

			ros::Time module_next_arrival(sum_seconds, sum_microsseconds*1000UL);

			if(ros::Time::now() >= module_next_arrival && !modules_iterator->second.isActive()) {

				sync = false;

				updateParameters(modules_iterator, module_next_arrival);

			}	
		}

		//ros::spinOnce();
		//std::cout << "[checkForDeadlineUpdate] Finished Checking for Updates..." << std::endl;
		loop_rate.sleep();
	}

}

void SchedulerModule::updateParameters(std::map<std::string, ModuleParameters>::iterator modules_iterator, ros::Time module_next_arrival) {
		//std::cout << "updating Parameters" << std::endl;
		//std::cout << "Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl;
		//std::cout << "[updateParameters] Updating Parameters..." << std::endl;
        ros::Duration difference;

        difference = ros::Time::now() - module_next_arrival;

        int aux;

        aux = static_cast<int>((difference.sec*1000000UL + difference.nsec/1000UL)/((1/modules_iterator->second.getFrequency())*1000000UL));

        aux += 1;

        modules_iterator->second.setTaskCounter(modules_iterator->second.getTaskCounter()[0],1);
        modules_iterator->second.setTaskCounter(modules_iterator->second.getTaskCounter()[0]+aux, 0);

		uint32_t sum_seconds = modules_iterator->second.getAbsoluteDeadline().sec;
        uint32_t d;

        d = static_cast<uint32_t>((modules_iterator->second.getTaskCounter()[0] - modules_iterator->second.getTaskCounter()[1])*((1/modules_iterator->second.getFrequency())*1000000UL));

        uint32_t sum_microsseconds = static_cast<uint32_t>(modules_iterator->second.getAbsoluteDeadline().nsec/1000UL) + d;

        while(sum_microsseconds >= 1000000UL) {
			sum_seconds++;
			sum_microsseconds -= 1000000UL;
		}

        //Assigning new deadline value
        ros::Time new_deadline(sum_seconds, sum_microsseconds*1000UL);

        {
        	std::unique_lock< std::mutex > lk( _ready_queue_sync );
        	modules_iterator->second.setAbsoluteDeadline(new_deadline);
        	modules_iterator->second.setExecutedInCycle(false);
        	modules_iterator->second.setTaskCounter(modules_iterator->second.getTaskCounter()[0] - 1, 1);

        	//std::cout << "Inserting in ready_queue" << std::endl;
        	std::pair<std::string, ros::Time> item = std::make_pair(modules_iterator->first, new_deadline);

        	std::set<std::pair<std::string, ros::Time>, Comparator>::iterator ready_queue_iterator;

        	ready_queue.insert(item);
        	//std::cout << "Inserted new element" << std::endl;

        	sync = true;
        	ready.notify_one();

        	lk.unlock();
        }

        //std::cout << "[updateParameters] Finished Updating Parameters..." << std::endl;

}

void SchedulerModule::coordinateModules() {

	ros::Rate loop_rate(frequency);

	//ros::spinOnce();

	while(ros::ok()) {
	//std::cout << "[coordinateModules] Checking Modules..." << std::endl;
	unsigned int i = 0;

	std::vector<std::string>::iterator finished_iterator;
	std::map<std::string, ModuleParameters>::iterator modules_iterator;

	while( i < scheduling_queue.size() && ros::ok()) {
		//std::cout << "Entering coordinateModules loop" << std::endl;
		//std::cout << "Module name: " << scheduling_queue[i] << std::endl;

		/*rs_messages::Finish msg;
        msg.name = "";
        msg.sec = 0;
        msg.nsec = 0;*/
        messages::ReconfigurationCommand reconfig;
        reconfig.source = ros::this_node::getName();
        reconfig.target = "";
        reconfig.action = "";

        //bool invalid = false;

        finished_iterator = std::find(finished_modules.begin(), finished_modules.end(), scheduling_queue[i]);
		if(finished_iterator != finished_modules.end()) {
			//std::cout << "Module Finished" << std::endl;
			//std::cout << "max_time: " << boost::posix_time::to_iso_extended_string(scheduling_modules[scheduling_queue[i]].max_time.toBoost()) << std::endl;
			//std::cout << "Module Name: " << scheduling_queue[i] << std::endl;

			if(scheduling_modules[scheduling_queue[i]].getFinishTime() > scheduling_modules[scheduling_queue[i]].getMaxTime()) {
			    {
                    std::unique_lock< std::mutex > lock( _modules_mutex );
                    deleting.wait(lock, [this]{return deleting_sync;});
                }

				/*
	            std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	   			publisher_iterator = schedule_pub.find(scheduling_queue[i]);

	            if(publisher_iterator != schedule_pub.end()) {
			        schedule_pub[scheduling_queue[i]].publish(reconfig);
			    } else {
			        invalid = true;
			    }
			    //if(!invalid) {
			        //ros::spinOnce();
			    //}
				*/
			}

			modules_iterator = connected_modules.find(scheduling_queue[i]);

		    if(modules_iterator != connected_modules.end()) {
		        modules_iterator->second.setExecutedInCycle(true);
		        modules_iterator->second.setActive(false);
				
				schedule_pub.publish(reconfig);
		    }

		    finished_modules.erase(finished_iterator);
			scheduling_queue.erase(scheduling_queue.begin() + i);

		} else {

			scheduling_modules[scheduling_queue[i]].setDiff(ros::Time::now() - scheduling_modules[scheduling_queue[i]].getInit());

			if(scheduling_modules[scheduling_queue[i]].getDiff() >= timeout_time || ros::Time::now() > scheduling_modules[scheduling_queue[i]].getMaxTime()) {
				scheduler_record << "Module Timeout" << std::endl;
				scheduler_record << "Module Name: " << scheduling_queue[i] << std::endl;
				scheduler_record << "Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl << std::endl;
				{
			        std::unique_lock< std::mutex > lock( _modules_mutex );

	                /*std::map<std::string, ros::Publisher>::iterator publisher_iterator;
	   				publisher_iterator = schedule_pub.find(scheduling_queue[i]);

	                if(publisher_iterator != schedule_pub.end()) {
			            schedule_pub[scheduling_queue[i]].publish(reconfig);
						
			        }
					*/
			    }

			    modules_iterator = connected_modules.find(scheduling_queue[i]);

		        if(modules_iterator != connected_modules.end()) {
		            modules_iterator->second.setExecutedInCycle(true);
		            modules_iterator->second.setActive(false);

					schedule_pub.publish(reconfig);
		        }

		        scheduling_queue.erase(scheduling_queue.begin() + i);
			} else {
				//std::cout << "Module's running" << std::endl;
				i++;
			}

		}
	}
	//ros::spinOnce();
	//std::cout << "[coordinateModules] Finished Checking Modules..." << std::endl;
	loop_rate.sleep();
	}
}