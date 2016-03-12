
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
#include <boost/algorithm/string.hpp>

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

bool ExplorationRemoteController::sendControlMessage(std::string target, adhoc_communication::ExpControl control_to_send) {

	std::string topic = "exp_control";

	adhoc_communication::SendExpControl service_control; // create request of type any+
    
    ROS_INFO("sending auction to multicast group on topic '%s'", topic.c_str());
    service_control.request.dst_robot = target; 
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

       
std::vector<std::string> parseCommand(std::string line) {
	std::vector<std::string> parts;
	boost::split(parts, line, boost::is_any_of("\t "));

	std::vector<std::string> strs;
	for (int i=0; i < parts.size(); i++) {
		std::string part = parts[i];
		if (part.length() <= 0) {
			continue;
		} 
    	strs.push_back(part);
    }

	for (int i=0; i < strs.size(); i++) {
    	std::cout << strs[i] << std::endl;
    }
    return strs;
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
    	std::cout << std::endl << "Enter command: " << std::endl;
        ros::spinOnce();
		adhoc_communication::ExpControl control_msgs;
		std::string action = "";
        std::string target = "mc_";
        int distance = -1;

        std::vector<std::string> parts = parseCommand(line);
        if (parts.size() <= 0) {
        	continue;
        }
		
		int index = 0;
		
        action = parts[index];
        if (action.compare("start") == 0) {
        	control_msgs.action = EXP_CONTROL_START;
	        std::cout << "Start: ";
        } else if (action.compare("stop") == 0) {
        	control_msgs.action = EXP_CONTROL_STOP;
	        std::cout << "Stop: ";
        } else {
        	std::cout << "unkown action: " << action; 
        	continue;
        }

        index++;
        if (parts.size() >= index + 1) {
    		if (parts[index].compare("all") == 0) {
    			target = "mc_";
    		} else {
    			target = parts[index];
    		}
    		index++;
        }	

        if (target.compare("mc_") == 0) {
        	std::cout << "all ";
        } else {
        	std::cout << target << " ";	
        }

        // distance
        if (parts.size() >= index + 1 && control_msgs.action != EXP_CONTROL_STOP) {
        	try {
				distance = atoi(parts[index].c_str());
    			std::cout << "Distance: " << distance;
        	} catch(...) {}
        }
        control_msgs.exploreDistanceFromHome = distance;
        index++;

    	controller.sendControlMessage(target, control_msgs);
    }
    return 0;
}