#include "ros/ros.h"
#include "tf/tf.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <std_srvs/Trigger.h>


void activateDirtDetection()
{
	// prepare the request and response messages
	std_srvs::Trigger::Request req;
	std_srvs::Trigger::Response res;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/activate_dirt_detection", req, res);
	assert(success == res.success);

	if (res.success == true)
		std::cout << "Dirt detection successfully activated.\n" << std::endl;
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}

void deactivateDirtDetection()
{
	// prepare the request and response messages
	std_srvs::Trigger::Request req;
	std_srvs::Trigger::Response res;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call("/dirt_detection/deactivate_dirt_detection", req, res);
	assert(success == res.success);

	if (res.success == true)
		std::cout << "Dirt detection successfully deactivated.\n" << std::endl;
	else
		std::cout << "The service call was not successful.\n" << std::endl;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "dirt_detection_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = true;
	serviceAvailable &= ros::service::waitForService("/dirt_detection/activate_dirt_detection", 5000);
	serviceAvailable &= ros::service::waitForService("/dirt_detection/deactivate_dirt_detection", 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The service could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service server is advertised.\n" << std::endl;

	// show menu
	char key = '0';
	while(key != 'q')
	{
		std::cout << "\n\nDirt detection\n\n 1. dirt detection on\n 2. dirt detection off\n q. quit\n\nChoose for an option: ";
		std::cin >> key;
		std::cout << "\n\n";

		if (key == '1')
			activateDirtDetection();
		else if (key == '2')
			deactivateDirtDetection();
	}


	return 0;
}
