#ifndef EXPLORER_H___
#define EXPLORER_H___

#include "ros/ros.h"
#include <Explorer.h>
#include "Explorer.h"
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
#include <Config.h>
#include "Config.h"

namespace explorer {

class Explorer {
	public:

  struct map_progress_t
        {
            double local_freespace;
            double global_freespace;
            double time;
        } map_progress;
        
	ros::Subscriber sub_move_base, sub_obstacle;    
        
	// create a costmap
	costmap_2d::Costmap2DROS* costmap2d_local;
        costmap_2d::Costmap2DROS* costmap2d_local_size;
	costmap_2d::Costmap2DROS* costmap2d_global;
	costmap_2d::Costmap2D costmap;

        std::vector<map_progress_t> map_progress_during_exploration;
        
        std::vector<int> clusters_available_in_pool;
        
        int home_position_x, home_position_y;
        int robot_id, number_of_robots;
        int frontier_selection, costmap_width, global_costmap_iteration, number_unreachable_frontiers_for_cluster;
        int counter_waiting_for_clusters;
        
        double robot_home_position_x, robot_home_position_y, costmap_resolution;
        bool Simulation, goal_determined;
        bool robot_prefix_empty;
        int accessing_cluster, cluster_element_size, cluster_element;
        int global_iterattions;
        bool cluster_flag, cluster_initialize_flag;
        int global_iterations_counter; 
        int waitForResult;
        std::string move_base_frame;
        std::string robot_prefix;               /// The prefix for the robot when used in simulation
        std::string robot_name;
        const unsigned char* occupancy_grid_global;
        const unsigned char* occupancy_grid_local;
        
        std::string csv_file, log_file;
        std::string log_path;
        std::fstream fs_csv, fs;


		Explorer(config::Config & c, tf::TransformListener& tf);
		void explore(int exploreDistanceFromHome);
		void frontiers();
		void map_info();
		int global_costmap_size();
		int local_costmap_size();
		void initLogPath();
		void save_progress(bool final=false);
		void exploration_has_finished();
		void indicateSimulationEnd();
		bool iterate_global_costmap(std::vector<double> *global_goal, std::vector<std::string> *robot_str);
		bool navigate(std::vector<double> goal);
		bool navigateHome();
		void visualize_goal_point(double x, double y);
		void visualize_home_point();
		bool move_robot(int seq, double position_x, double position_y);
		bool turn_robot(int seq);
		void feedbackCallback(
			const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
		bool target_reached(void);
		void storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
		bool checkIfFrontierIsReachable(double x, double y);
		void setExplorationFinished(bool b);
		int calculatePlanDistance(double startX, double startY, double goalX, double goal);

		ros::Publisher pub_move_base;
		ros::Publisher pub_Point;
		ros::Publisher pub_home_Point;
	        ros::Publisher pub_frontiers;

	        ros::ServiceClient mm_log_client;
        
	        ros::NodeHandle nh;
	        ros::Time time_start;

		//Create a move_base_msgs to define a goal to steer the robot to
		move_base_msgs::MoveBaseActionGoal action_goal_msg;
		move_base_msgs::MoveBaseActionFeedback feedback_msgs;

		//move_base::MoveBase simple_move_base;
		geometry_msgs::PointStamped goalPoint;
		geometry_msgs::PointStamped homePoint;

		std::vector<geometry_msgs::PoseStamped> goals;
		tf::Stamped<tf::Pose> robotPose;

		explorationPlanner::ExplorationPlanner *exploration;       
	        
		double x_val, y_val, home_point_x, home_point_y;
		int seq, feedback_value, feedback_succeed_value, rotation_counter,
				home_point_message, goal_point_message;
		int counter;
		bool pioneer, exploration_finished, running, backToHome;

	};

}
#endif
