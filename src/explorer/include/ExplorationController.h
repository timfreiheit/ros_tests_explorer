#ifndef CONTROLLER_H___
#define CONTROLLER_H___

#include <ExplorationPlanner.h>
#include "Explorer.h"
#include <Explorer.h>
#include <navfn/navfn_ros.h>
#include "ros/ros.h"
#include "hungarian.h"
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <adhoc_communication/ExpFrontier.h> //<simple_navigation/Frontier.h>
#include <adhoc_communication/ExpCluster.h>
#include <adhoc_communication/ExpAuction.h>
#include <adhoc_communication/MmPoint.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <map_merger/TransformPoint.h>
#include <Config.h>
#include "Config.h"
//#include <dynamic_reconfigure/server.h>

namespace explorationController {

	class ExplorationController {
		public:
			ExplorationController(config::Config& c_, explorer::Explorer& target);
			void explore();
		private:
			config::Config* c;
			explorer::Explorer* explorer;

			ros::NodeHandle nh_control;
			ros::Subscriber sub_control;

			void registerAdHocCommunication();
	};

}
#endif
