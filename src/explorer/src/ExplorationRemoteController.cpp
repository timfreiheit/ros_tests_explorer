
#include "ExplorationRemoteController.h"
#include "Config.h"
#include <Config.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <adhoc_communication/SendExpControl.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <navfn/navfn_ros.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "adhoc_communication/MmPoint.h"

using namespace explorationRemoteController;


ExplorationRemoteController::ExplorationRemoteController(config::Config& c_) {
	c = &c_;

	std::string prefix = c->adhocCommunicationTopicPrefix();
	std::string sendExpControl_msgs = prefix + "/adhoc_communication/send_exp_control";
    
    ROS_DEBUG("Sending exploration control: '%s'", sendExpControl_msgs.c_str());
    
    ssendExpControl = nh_service.serviceClient<adhoc_communication::SendExpControl>(sendExpControl_msgs);

}

void ExplorationRemoteController::run() {
    registerCallbacks();
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
}

void ExplorationRemoteController::registerCallbacks() {
    sub_all_positions = nh_service.subscribe("all_positions", 99999, &ExplorationRemoteController::allPositionsCallback, this);
    sub_robot_positions = nh_service.subscribe("map_merger/position_robot_0", 99999, &ExplorationRemoteController::robotPositionsCallback, this);
}

void ExplorationRemoteController::allPositionsCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg) {
    /*ROS_ERROR(" RECEIVED POSITIONS MESSAGE ");
    std::cout << "allPositionsCallback: ";

    std::vector<adhoc_communication::MmPoint> points = msg->positions;
    std::cout << " Points: " << points.size() << "\n";

    std::map<std::string, std::vector<adhoc_communication::MmPoint>> map;
    for (int i=0, size = points.size(); i<size ; i++) {
        adhoc_communication::MmPoint point = points[i];
        if (!map.count(point.src_robot)) {
            std::vector<adhoc_communication::MmPoint> robot_points;
            map[point.src_robot] = robot_points;
        }
        map[point.src_robot].push_back(point);
        std::cout << " Points: " << point.x << " y: " << point.y << "\n";
    }

    for(std::map<std::string,std::vector<adhoc_communication::MmPoint>>::iterator iter = map.begin(); iter != map.end(); ++iter){
        std::string k =  iter->first;
        std::vector<adhoc_communication::MmPoint> v = iter->second;
        int size = v.size();
        std::cout << " Robot " << k << " position_size: " << size << "\n";
    }*/
}

void ExplorationRemoteController::robotPositionsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    //ROS_ERROR(" RECEIVED POSITION MESSAGE ");
    //std::cout << "robotPositionsCallback: " + msg->markers.size();
}

bool ExplorationRemoteController::sendControlMessage(std::string target, adhoc_communication::ExpControl control_to_send) {

	std::string topic = "exp_control";

	adhoc_communication::SendExpControl service_control; // create request of type any+
    
    ROS_INFO("\n");
    service_control.request.dst_robot = target; 
    service_control.request.control = control_to_send;
    service_control.request.topic = topic;

    if (ssendExpControl.call(service_control)) {
        ROS_ERROR("Successfully called service sendToMulticast");

        if(service_control.response.status) {
            ROS_ERROR("Control was multicasted successfully.");
            return true;
        } else {
            ROS_WARN("Failed to send control to mutlicast group!");
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

    boost::thread thr_map(boost::bind(&ExplorationRemoteController::run, &controller));

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
        } else if (action.compare("pause") == 0) {
        	control_msgs.action = EXP_CONTROL_PAUSE;
	        std::cout << "Pause: ";
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