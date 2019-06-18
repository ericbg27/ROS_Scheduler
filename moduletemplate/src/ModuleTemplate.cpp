#include "ModuleTemplate.hpp"

ModuleTemplate::ModuleTemplate(const int32_t &argc, char **argv) : 
	md(),
	received_name(""),
	topic_name(""),
	check_frequency(2) {}

ModuleTemplate::~ModuleTemplate() {}

void schedulingSigIntHandler(int sig) {
	/********************************************************************************
	Description: Basic Module SigInt Handler for module disconnection

	-> Note: Object erasing MUST be done in a tearDown() function
	-> Note2: If module termination is not done via SigINT, this functionality has to be
	passed to tearDown() function

	@param: int sig - Signal for CTRL+C event
	*********************************************************************************/

	//Setting request parameters with connection = false
	services::SchedulerServerData srv;
	srv.request.name = ros::this_node::getName();
	srv.request.frequency = 0;
	srv.request.deadline = 0;
	srv.request.wce = 0;

	srv.request.connection = false;

	ros::NodeHandle client_handler, param_handler("~");

	ros::ServiceClient client_module;

	//Connection to scheduler module management service
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	if(client_module.call(srv)) {
		ROS_INFO("Succesfully disconnected from scheduler.");
	} else {
		ROS_ERROR("Failed to disconnect from scheduler.");
	}

	ros::shutdown();
}

//void ModuleTemplate::tearDown() {}

void ModuleTemplate::setUp() {
	/********************************************************************************
	Description: Basic Module initialization function

	-> MUST be called before run()!!

	@param: None
	*********************************************************************************/

	/***************************************************** Mandatory functionality ************************************************/
	
	//Defining custom SIGINT Handler
	signal(SIGINT, schedulingSigIntHandler);

	ros::NodeHandle client_handler, param_handler("~");
	client_module = client_handler.serviceClient<services::SchedulerServerData>("ModuleManagement");

	//Setting up module descriptor and connection request
	md.name = ros::this_node::getName();
	param_handler.getParam("frequency", md.frequency);

	int deadline_param, wce_param;

	//Getting parameters defined by user
	//Note: They must be defined for proper functioning
	param_handler.getParam("deadline", deadline_param);
	param_handler.getParam("wce", wce_param);
	md.deadline = static_cast<uint32_t>(deadline_param);
	md.wce = static_cast<uint32_t>(wce_param);

	param_handler.getParam("check_frequency", check_frequency);

	md.connection = true;

	services::SchedulerServerData srv;
	srv.request.name = md.name;
	srv.request.frequency = md.frequency;
	srv.request.deadline = md.deadline;
	srv.request.wce = md.wce;
	srv.request.connection = md.connection;

	if(client_module.call(srv)) {
		ROS_INFO("Succesfully connected to scheduler.");
		connected = true;
	} else {
		ROS_ERROR("Failed to connect to scheduler.");
	}

	ros::NodeHandle scheduling_handler, finish_scheduling_handler;

	//Subscrbing to this module's scheduling topic
	topic_name = ros::this_node::getName().substr(1) + "topic";
    sched = scheduling_handler.subscribe(topic_name, 1, &ModuleTemplate::schedulingCallback, this);

    //Publishing in finish topic, which indicates end of module's execution
	scheduling_pub = finish_scheduling_handler.advertise<std_msgs::String>("scheduling_finish", 1);

	/*****************************************************************************************************************************/
	
	/*******************************
	Custom functionality comes here
	*******************************/
}

void ModuleTemplate::schedulingCallback(const std_msgs::StringConstPtr& msg) {
	/********************************************************************************
	Description: Callback to receive execution command from scheduler

	-> Note: If module receives it's name, execution is allowed

	@param: const std_msgs::StringConstPtr& msg - name received
	*********************************************************************************/
	received_name = msg->data;
}

void ModuleTemplate::run() {
	/********************************************************************************
	Description: Basic module run function

	-> Note: Module functionality goes inside if

	@param: None
	*********************************************************************************/

	//Defining module's execution minimum checking period
	float period = static_cast<float>(md.deadline)/1000000.0;

	//Defining checking frequency
	//Note: By default it is 2/period
	//Note2: The bigger check_frequency is, higher the granularity
	ros::Rate loop_rate((1/period)*check_frequency);

	while(!ros::isShuttingDown()) {
		//receive name
		ros::spinOnce();

		if(received_name == md.name) {
			//Basic printing, can be uncommented if desirable
			//ROS_INFO("Running now");

			/*******************************
			Custom functionality comes here
			*******************************/

			std_msgs::String msg;

			msg.data = md.name;

			//Publishing in scheduling_finish topic
			scheduling_pub.publish(msg);

			ros::spinOnce();

			//Sleep until next check
			loop_rate.sleep();
		} else {
			//Basic printing, can be uncommented if desirable
			//ROS_INFO("Not Running");

			//Sleep until next check
			loop_rate.sleep();
		}

		received_name = "";
	}

	//If tearDown() function exists, uncomment next line
	//return tearDown()

}