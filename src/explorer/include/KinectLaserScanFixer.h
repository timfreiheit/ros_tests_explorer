#ifndef KINECT_LASER_SCAN_FIXER_H___
#define KINECT_LASER_SCAN_FIXER_H___

#include "ros/ros.h"
#include <ros/console.h>
#include <std_msgs/String.h>
#include <Config.h>
#include "Config.h"
#include "Constants.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/LaserScan.h>


namespace kinectLaserScanFixer {

	class KinectLaserScanFixer {
		public:
			void run();
			void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		private:
            ros::NodeHandle nh_service;
			ros::Subscriber sub_scan;
			ros::Publisher publish_scan;
			void registerCallbacks();
	};

}

#endif
