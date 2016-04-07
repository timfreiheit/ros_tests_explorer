#ifndef EXPLORATION_REMOTE_CONTROLLER_H___
#define EXPLORATION_REMOTE_CONTROLLER_H___

#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/String.h>
#include <Config.h>
#include "Config.h"
#include "Constants.h"
#include <adhoc_communication/SendExpControl.h>
#include <adhoc_communication/MmListOfPoints.h>
#include "visualization_msgs/MarkerArray.h"


namespace explorationRemoteController {

	class ExplorationRemoteController {
		public:
			ExplorationRemoteController(config::Config& c);
			bool sendControlMessage(std::string target, adhoc_communication::ExpControl control_to_send);
			void allPositionsCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg);
			void robotPositionsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
			void run();
		private:
			config::Config* c;
            ros::NodeHandle nh_service;
			ros::Subscriber sub_robot_positions;
			ros::Subscriber sub_all_positions;
            ros::ServiceClient ssendExpControl;

			void registerCallbacks();
	};

}

#endif
