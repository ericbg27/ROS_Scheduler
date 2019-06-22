#include "TestModule.hpp"

TestModule::TestModule(const int32_t &argc, char **argv) : 
	md(),
	received_name(""),
	topic_name("") {}

TestModule::~TestModule() {}

void schedulingSigIntHandler(int sig) {
	services::SchedulerServerData srv;
	srv.request.name = ros::this_node::getName();
	srv.request.frequency = 0;
	srv.request.deadline = 0;
	srv.request.wce = 0;

	srv.request.connection = false;

	ros::NodeHandle client_handler, param_handler("~");

	ros::ServiceClient client_module;
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	if(client_module.call(srv)) {
		ROS_INFO("Succesfully disconnected from scheduler.");
	} else {
		ROS_ERROR("Failed to disconnect from scheduler.");
	}

	ros::shutdown();
}

void TestModule::setUp() {

	signal(SIGINT, schedulingSigIntHandler);
	ros::NodeHandle client_handler, param_handler("~");
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	md.name = ros::this_node::getName();
	param_handler.getParam("frequency", md.frequency);

	int deadline_param, wce_param;
	param_handler.getParam("deadline", deadline_param);
	param_handler.getParam("wce", wce_param);
	md.deadline = static_cast<uint32_t>(deadline_param);
	md.wce = static_cast<uint32_t>(wce_param);

	md.connection = true;

	services::SchedulerServerData srv;
	srv.request.name = md.name;
	srv.request.frequency = md.frequency;
	srv.request.deadline = md.deadline;
	srv.request.wce = md.wce;
	srv.request.connection = md.connection;

	if(client_module.call(srv)) {
		ROS_INFO("Succesfully connected to scheduler.");
	} else {
		ROS_ERROR("Failed to connect to scheduler.");
	}

	ros::NodeHandle scheduling_handler, finish_scheduling_handler;

	topic_name = ros::this_node::getName().substr(1) + "topic";
    sched = scheduling_handler.subscribe(topic_name, 1, &TestModule::schedulingCallback, this);

    finish_topic_name = topic_name + "_finish";
	scheduling_pub = finish_scheduling_handler.advertise<rs_messages::FinishMessage>(finish_topic_name, 1);
}

//void TestModule::tearDown() {}

void TestModule::schedulingCallback(const std_msgs::StringConstPtr& msg) {
	std::cout << "Time: " << boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()) << std::endl;
	std::cout << "received_name: " << msg->data << std::endl;
	received_name = msg->data;
}

void TestModule::run() {

	float period = static_cast<float>(md.deadline)/1000000.0;

	ros::Rate loop_rate((1/period)*5);

	while(!ros::isShuttingDown()) {
		ros::spinOnce();

		if(received_name == md.name) {
			ROS_INFO("Running now");

			ros::Time finish_time = ros::Time::now();

			rs_messages::FinishMessage msg;

			msg.name = ros::this_node::getName();
			msg.sec = finish_time.sec;
			msg.nsec = finish_time.nsec;

			scheduling_pub.publish(msg);	

			loop_rate.sleep();
		} else {
			//ROS_INFO("Not Running");
			loop_rate.sleep();
		}

		received_name = "";
	}

}