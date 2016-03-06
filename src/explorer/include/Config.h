#ifndef CONFIG_H___
#define CONFIG_H___

#include "ros/ros.h"
#include <ros/console.h>
#include <boost/lexical_cast.hpp>
#include <move_base/MoveBaseConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <std_msgs/String.h>
//#include <costmap_2d/voxel_costmap_2d.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
//#include <sound_play/sound_play.h>
#include <boost/filesystem.hpp>
#include <map_merger/LogMaps.h>
#include <nav_msgs/GetPlan.h>

namespace config {

	class Config {
		public:
        	std::string robot_prefix; 
        	std::string robot_name;  
        	int robot_id;  
    		bool robot_prefix_empty;
        
			Config();
		private:
			ros::NodeHandle nh;
			void setUpRobotId();
	};

}

#endif
