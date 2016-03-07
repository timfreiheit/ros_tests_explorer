#ifndef EXPLORATION_REMOTE_CONTROLLER_H___
#define EXPLORATION_REMOTE_CONTROLLER_H___

#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/String.h>
#include <Config.h>
#include "Config.h"

namespace explorationRemoteController {

	class ExplorationRemoteController {
		public:
			ExplorationRemoteController(config::Config& c);
		private:
			config::Config* c;
            ros::NodeHandle *nh_service;
            ros::ServiceClient ssendExpControl;
	};

}

#endif
