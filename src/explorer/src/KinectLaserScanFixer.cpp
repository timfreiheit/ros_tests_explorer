
#include "KinectLaserScanFixer.h"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>     // std::cout
#include <algorithm>    // std::replace_if
#include <vector>       // std::vector

#include <boost/thread.hpp>

using namespace kinectLaserScanFixer;

void KinectLaserScanFixer::run() {
    registerCallbacks();
}

void KinectLaserScanFixer::registerCallbacks() {
    sub_scan = nh_service.subscribe("scan", 99999, &KinectLaserScanFixer::scanCallback, this);
    publish_scan = nh_service.advertise<sensor_msgs::LaserScan>("scan_fix", 1000);
}

bool isNaN (float i) { return i != i; }

void KinectLaserScanFixer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    std::cout << " get scan topic";

    sensor_msgs::LaserScan scan;
    scan.header = msg->header;
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.intensities = msg->intensities;
    scan.ranges = msg->ranges;

    std::replace_if (scan.ranges.begin(), scan.ranges.end(), isNaN, (float) 15);
    std::replace_if (scan.intensities.begin(), scan.intensities.end(), isNaN, (float) 15);
    
    publish_scan.publish(scan);
}


 
int main(int argc, char **argv) {
	/*
	 * ROS::init() function needs argc and argv to perform
	 * any argument and remapping that is provided by the
	 * command line. The third argument is the name of the node
	 */
	ros::init(argc, argv, "kinectLaserScanFixer");

	kinectLaserScanFixer::KinectLaserScanFixer fixer;

    fixer.run();

    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    // TODO 

    return 0;
}