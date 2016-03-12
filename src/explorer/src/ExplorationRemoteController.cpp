
#include "ExplorationRemoteController.h"
#include "Config.h"
#include <Config.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <adhoc_communication/SendExpControl.h>
#include <navfn/navfn_ros.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

using namespace explorationRemoteController;


ExplorationRemoteController::ExplorationRemoteController(config::Config& c_) {
	c = &c_;

	std::string prefix = c->adhocCommunicationTopicPrefix();
	std::string sendExpControl_msgs = prefix + "/adhoc_communication/send_exp_control";
    
    
    ros::NodeHandle tmp;
    nh_service = &tmp;
    
    ROS_DEBUG("Sending exploration control: '%s'", sendExpControl_msgs.c_str());
    
    ssendExpControl = nh_service->serviceClient<adhoc_communication::SendExpControl>(sendExpControl_msgs);
}

bool ExplorationRemoteController::sendControlMessage(adhoc_communication::ExpControl control_to_send) {

	std::string topic = "exp_control";

	adhoc_communication::SendExpControl service_control; // create request of type any+
    
    ROS_INFO("sending auction to multicast group on topic '%s'", topic.c_str());
    service_control.request.dst_robot = "mc_"; 
    service_control.request.control = control_to_send;
    service_control.request.topic = topic;

    if (ssendExpControl.call(service_control)) {
        ROS_ERROR("Successfully called service sendToMulticast");

        if(service_control.response.status) {
            ROS_ERROR("Auction was multicasted successfully.");
            return true;
        } else {
            ROS_WARN("Failed to send auction to mutlicast group!");
            return false;
        }                  
    } else {
     	ROS_WARN("Failed to call service sendToMulticastAuction [%s]",ssendExpControl.getService().c_str());
     	return false;
    }
}

            
int main(int argc, char **argv) {
	/*
	 * ROS::init() function needs argc and argv to perform
	 * any argument and remapping that is provided by the
	 * command line. The third argument is the name of the node
	 */
	ros::init(argc, argv, "explorationRemoteController");

	config::Config config;
	explorationRemoteController::ExplorationRemoteController controller(config);

	int action = EXP_CONTROL_START;
    ros::spinOnce();
 	for (std::string line; std::getline(std::cin, line);) {
        ros::spinOnce();
    	std::cout << line << std::endl;

    	adhoc_communication::ExpControl control_msgs;	
        control_msgs.action = action;
        control_msgs.exploreDistanceFromHome = 5;

        if (action == EXP_CONTROL_START) {
        	std::cout << "Start exploration";
        	action = EXP_CONTROL_STOP;
        } else {
        	std::cout << "Stop exploration";
        	action = EXP_CONTROL_START;
        }

    	controller.sendControlMessage(control_msgs);
    }
    return 0;
}