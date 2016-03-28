
#include "ExplorationController.h"
#include "ros/ros.h"
#include "ExplorationPlanner.h"
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
#include "Config.h"
#include "Constants.h"

using namespace explorationController;


ExplorationController::ExplorationController(config::Config& c_, explorer::Explorer& target) {
	explorer = &target;
	c = &c_;

//	boost::thread thr_explore(boost::bind(&ExplorationController::explore, this));	

	registerAdHocCommunication();
}


void ExplorationController::explore() {
    	ROS_ERROR("----------------  Start explore ----------------------------");
	explorer->explore(exploreDistanceFromHome);
}

void ExplorationController::registerAdHocCommunication() {
	sub_control = nh_control.subscribe(c->adhocCommunicationTopicPrefix()+"/exp_control", 10000, &ExplorationController::controlCallback, this);
}

void ExplorationController::controlCallback(const adhoc_communication::ExpControl::ConstPtr& msg) {
    ROS_ERROR("----------------  RECEIVED CONTROL MESSAGE ----------------------------");
    ROS_ERROR("----------------  STATUS: Running: %d ----------------------------", explorer->running);
    if (msg.get()->action == EXP_CONTROL_START) {
    	ROS_ERROR("----------------  MESSAGE: START EXPLORATION ---------------------------- distance: %d", msg.get()->exploreDistanceFromHome);
    	if (explorer->running == false) {
    		exploreDistanceFromHome = msg.get()->exploreDistanceFromHome;
			boost::thread thr_explore(boost::bind(&ExplorationController::explore, this));	
			explorer->running = true;
    	} else {
    		explorer->exploration->exploreDistanceFromHome = msg.get()->exploreDistanceFromHome;
    	}

    } else if (msg.get()->action == EXP_CONTROL_STOP) {
    	ROS_ERROR("----------------  MESSAGE: STOP EXPLORATION ----------------------------");
    	if (explorer->running == false) {
    		// start explorer only to drive home
    		exploreDistanceFromHome = 0;
			boost::thread thr_explore(boost::bind(&ExplorationController::explore, this));	
			explorer->running = true;
    	}
    	explorer->setExplorationFinished(true);
    	explorer->backToHome = true;
    } else if (msg.get()->action == EXP_CONTROL_PAUSE) {
    	ROS_ERROR("----------------  MESSAGE: PAUSE EXPLORATION ----------------------------");
    	explorer->exploration_finished = true;
    	explorer->backToHome = false;		
    }
}

