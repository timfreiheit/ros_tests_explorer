#include "Config.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/String.h>

using namespace config;

Config::Config(): nh("~") {
	nh.param<std::string>("robot_prefix",robot_prefix,"");
    ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

    setUpRobotId();
	ROS_ERROR("Robot: %d", robot_id);

}

void Config::setUpRobotId() {

	ROS_ERROR("[CONFIG]: setUpRobotId");

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

        if(robot_name.compare("turtlebot_01") == 0) {
        	robot_id = 5;
        }

        if(robot_name.compare("turtlebot_02") == 0) {
        	robot_id = 6;
        }

        if(robot_name.compare("turtlebot_03") == 0) {
        	robot_id = 7;
        }

        if(robot_name.compare("turtlebot_04") == 0) {
        	robot_id = 8;
        }

        if(robot_name.compare("turtlebot_05") == 0) {
        	robot_id = 9;
        }

        if(robot_name.compare("turtlebot_06") == 0) {
        	robot_id = 10;
        }

        if(robot_name.compare("turtlebot_07") == 0) {
        	robot_id = 11;
        }

        if(robot_name.compare("turtlebot_08") == 0) {
        	robot_id = 12;
        }

        if(robot_name.compare("turtlebot_09") == 0) {
        	robot_id = 13;
        }

        if(robot_name.compare("turtlebot_10") == 0) {
        	robot_id = 14;
        }

        if(robot_name.compare("qbot") == 0) {
        	robot_id = 15;
        }
               
        robot_prefix_empty = true;
        ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
    } else {
        if(robot_name.compare("BASE") == 0) {
        	robot_id = -1;
        } else {
    		robot_name = robot_prefix;
    		ROS_INFO("robot_prefix: %s",robot_prefix.c_str());               
    		robot_id = atoi(robot_prefix.substr(7,1).c_str());
		}
		robot_id++;
		robot_prefix_empty = false;
    } 
}


std::string Config::adhocCommunicationTopicPrefix() {
/*
    std::stringstream robot_number;
    robot_number << robot_id;

    std::string prefix = "/robot_";
    std::string robo_name = prefix.append(robot_number.str());   
    */
    std::string robo_name = robot_prefix;
    if(robot_prefix_empty == true) {
        /*NO SIMULATION*/
        robo_name = "";
    }

    return robo_name; 
}