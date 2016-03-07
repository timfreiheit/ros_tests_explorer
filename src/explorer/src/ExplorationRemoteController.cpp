
#include "ExplorationRemoteController.h"
#include "Config.h"
#include <Config.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <adhoc_communication/SendExpControl.h>
#include <navfn/navfn_ros.h>

using namespace explorationRemoteController;


ExplorationRemoteController::ExplorationRemoteController(config::Config& c_) {
	c = &c_;

	std::string prefix = c->adhocCommunicationTopicPrefix();
	std::string sendExpControl_msgs = prefix + "/adhoc_communication/send_exp_control";
    
    /*
    ros::NodeHandle tmp;
    nh_service = &tmp;
    
    ROS_DEBUG("Sending exploration control: '%s'", sendExpControl_msgs.c_str());
    
    ssendExpControl = nh_service->serviceClient<adhoc_communication::SendExpControl>(sendExpControl_msgs);*/
}


            
int main(int argc, char **argv) {

}