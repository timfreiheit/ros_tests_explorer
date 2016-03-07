#ifndef CONFIG_H___
#define CONFIG_H___

#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/String.h>
#include <iostream>

namespace config {

	class Config {
		public:
        	std::string robot_prefix; 
        	std::string robot_name;  
        	int robot_id;  
    		bool robot_prefix_empty;
        
			Config();
			std::string adhocCommunicationTopicPrefix();
		private:
			ros::NodeHandle nh;
			void setUpRobotId();
	};

}

#endif
