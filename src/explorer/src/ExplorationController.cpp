
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

using namespace explorationController;


ExplorationController::ExplorationController(config::Config& c_, explorer::Explorer& target) {
	explorer = &target;
	c = &c_;

	boost::thread thr_explore(boost::bind(&ExplorationController::explore, this));	

	registerAdHocCommunication();
}


void ExplorationController::explore() {
	explorer->explore();
}

void ExplorationController::registerAdHocCommunication() {
	sub_control = nh_control.subscribe(c->adhocCommunicationTopicPrefix()+"/exp_control", 10000, &ExplorationController::controlCallback, this);
}

void ExplorationController::controlCallback(const adhoc_communication::ExpControl::ConstPtr& msg) {
    ROS_ERROR("----------------  RECEIVED CONTROL MESSAGE !!!!! ----------------------------");
}

