#include "Config.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <ExplorationPlanner.h>
#include <boost/lexical_cast.hpp>
#include <move_base/MoveBaseConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/cell_data.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation.h>
#include <costmap_2d/observation_buffer.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
//#include <costmap_2d/voxel_costmap_2d.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <navfn/navfn_ros.h>
//#include <sound_play/sound_play.h>
#include <boost/filesystem.hpp>
#include <map_merger/LogMaps.h>
#include <nav_msgs/GetPlan.h>
#include "Explorer.h"

using namespace config;

Config::Config(): nh("~") {
	nh.param<std::string>("robot_prefix",robot_prefix,"");
    ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

    setUpRobotId();

}

void Config::setUpRobotId() {

    std::string move_base_frame;
    nh.param<std::string>("move_base_frame",move_base_frame,"map"); 
	
	if(robot_prefix.empty()) {
        char hostname_c[1024];
        hostname_c[1023] = '\0';
        gethostname(hostname_c, 1023);
        robot_name = std::string(hostname_c);
            
        // '-' is not allows as ros node name
        std::replace( robot_name.begin(), robot_name.end(), '-', '_');
        ROS_INFO("NO SIMULATION! Robot name: %s", robot_name.c_str());

                                        
        /*
	     * THIS IS REQUIRED TO PERFORM COORDINATED EXPLORATION
         * Assign numbers to robot host names in order to make
         * auctioning and frontier selection UNIQUE !!!!!
         * To use explorer node on a real robot system, add your robot names 
         * here and at ExplorationPlanner::lookupRobotName function ... 
	     */
        std::string bob = "bob";
        std::string marley = "marley";
        std::string turtlebot = "turtlebot";
        std::string joy = "joy";
        std::string hans = "hans";
                    
        if(robot_name.compare(turtlebot) == 0) {
            robot_id = 0;
        }
        
        if(robot_name.compare(joy) == 0) {
            robot_id = 1;
        }
        
        if(robot_name.compare(marley) == 0) {
            robot_id = 2;
        }
        
        if(robot_name.compare(bob) == 0) {
            robot_id = 3;
        }
        
        if(robot_name.compare(hans) == 0) {
            robot_id = 4;
        }

        if(robot_name.compare("turtlebot_01")) {
        	robot_id = 5;
        }

        if(robot_name.compare("turtlebot_02")) {
        	robot_id = 6;
        }

        if(robot_name.compare("turtlebot_03")) {
        	robot_id = 7;
        }

        if(robot_name.compare("turtlebot_04")) {
        	robot_id = 8;
        }

        if(robot_name.compare("turtlebot_05")) {
        	robot_id = 9;
        }

        if(robot_name.compare("turtlebot_06")) {
        	robot_id = 10;
        }

        if(robot_name.compare("turtlebot_07")) {
        	robot_id = 10;
        }

        if(robot_name.compare("turtlebot_08")) {
        	robot_id = 11;
        }

        if(robot_name.compare("turtlebot_09")) {
        	robot_id = 12;
        }

        if(robot_name.compare("turtlebot_10")) {
        	robot_id = 13;
        }

        if(robot_name.compare("qbot")) {
        	robot_id = 14;
        }
               
        robot_prefix_empty = true;
        ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
    } else {
    	robot_name = robot_prefix;
    	ROS_INFO("Move_base_frame: %s",move_base_frame.c_str());               
    	robot_id = atoi(move_base_frame.substr(7,1).c_str());

	    ROS_INFO("Robot: %d", robot_id);
    } 
}