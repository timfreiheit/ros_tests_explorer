#include "ros/ros.h"
#include <Explorer.h>
#include "Explorer.h"
#include "ExplorationPlanner.h"
#include <ros/console.h>
#include <ExplorationPlanner.h>
#include "ExplorationController.h"
#include <ExplorationController.h>
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

#define forEach BOOST_FOREACH

//#define PROFILE
//#ifdef PROFILE
//#include <google/profiler.h>
//#include <google/heap-profiler.h>
//#endif
boost::mutex costmap_mutex;

#define OPERATE_ON_GLOBAL_MAP true		// global or local costmap as basis for exploration
#define OPERATE_WITH_GOAL_BACKOFF false	// navigate to a goal point which is close to (but not exactly at) selected goal (in case selected goal is too close to a wall)

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
    ros::Duration(1.0).sleep();
}

using namespace explorer;


    Explorer::Explorer(config::Config& c, tf::TransformListener& tf) :
        counter(0), rotation_counter(0), nh("~"), exploration_finished(false), number_of_robots(1), accessing_cluster(0), cluster_element_size(0),
        cluster_flag(false), cluster_element(-1), cluster_initialize_flag(false), global_iterattions(0), global_iterations_counter(0), 
        counter_waiting_for_clusters(0), global_costmap_iteration(0), robot_prefix_empty(false), robot_id(0), running(false), backToHome(false) {

        
                nh.param("frontier_selection",frontier_selection,1); 
                nh.param("local_costmap/width",costmap_width,0); 
                nh.param<double>("local_costmap/resolution",costmap_resolution,0);
                nh.param("number_unreachable_for_cluster", number_unreachable_frontiers_for_cluster,3);
                
                ROS_INFO("Costmap width: %d", costmap_width);
//                frontier_selection = atoi(param.c_str());
                ROS_INFO("Frontier selection is set to: %d", frontier_selection);
                //frontier_selection = 3;
                Simulation = false;      
                
//#ifdef PROFILE
//const char  fname[3] = "TS";
//ProfilerStart(fname);
//HeapProfilerStart(fname);
//#endif

                srand((unsigned)time(0));
                
//		sound_play::SoundClient sc;
//              sleepok(0.5, nh);
//		sc.say("Hello    World   I    am    Turtlebot");
		
                nh.param<std::string>("move_base_frame",move_base_frame,"map");  
                nh.param<int>("wait_for_planner_result",waitForResult,3);
                // determine host name
                nh.param<std::string>("robot_prefix",robot_prefix,"");
                ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

                //char hostname_c[1024];
                //hostname_c[1023] = '\0';
                //gethostname(hostname_c, 1023);
                //robot_name = std::string(hostname_c);
               
                // create map_merger service
                std::string service = robot_prefix + std::string("/map_merger/logOutput");
                mm_log_client = nh.serviceClient<map_merger::LogMaps>(service.c_str());
                
                robot_prefix_empty = c.robot_prefix_empty;
                robot_id = c.robot_id;
                robot_name = c.robot_name;
                /*
                 * If you want to operate only 1 robot but stage is operating 2
                 * then simply kill one of them.
                 */
//                if(robot_name == 1)
//                {
//                    ROS_ERROR("Shutting down ROBOT 1");
//                    ros::shutdown();
//                }
                
                
               /*
                *  CREATE LOG PATH
                * Following code enables to write the output to a file
                * which is localized at the log_path
                */
                initLogPath();
                csv_file = log_path + std::string("periodical.log");
                log_file = log_path + std::string("exploration.log"); 
                
		ROS_DEBUG("*********************************************");
		ROS_INFO("******* Initializing Simple Navigation ******");
		ROS_DEBUG("                                             ");

		ROS_INFO("Creating global costmap ...");
		costmap2d_global = new costmap_2d::Costmap2DROS("global_costmap", tf);
		ROS_INFO("Global costmap created ... now performing costmap -> pause");
		costmap2d_global->pause();
		ROS_INFO("Pausing performed");

		ROS_DEBUG("                                             ");

                if(OPERATE_ON_GLOBAL_MAP == true)
                {
                    costmap2d_local_size = new costmap_2d::Costmap2DROS("local_costmap", tf);
                    costmap2d_local_size->pause();
                    ROS_DEBUG("Starting Global costmap ...");
                    costmap2d_global->start();
                    costmap2d_local_size->start();
                    
                    costmap2d_local = costmap2d_global;                   
                }
                else
                {
                    ROS_INFO("Creating local costmap ...");
                    costmap2d_local = new costmap_2d::Costmap2DROS("local_costmap", tf);
                    ROS_INFO("Local costmap created ... now performing costmap -> pause");
                    costmap2d_local->pause();
                    ROS_INFO("Pausing performed");
                    ROS_INFO("Cost maps created");

                    ROS_INFO("                                             ");

                    ROS_INFO("Starting Global costmap ...");
                    costmap2d_global->start();
                    ROS_INFO("Starting Local costmap ... ");
                    costmap2d_local->start();
                    ROS_INFO("BOTH COSTMAPS STARTED AND RUNNING ...");
                }
		ROS_INFO("---------------- COSTMAP DONE ---------------");

		/*
		 * Set the first goal as PointStamped message to visualize in RVIZ.
		 * RVIZ requires a minimal history length of 1, which means that at least
		 * one entry has to be buffered, before the first Goal is able to be
		 * visualized. Therefore set the "first" goal to the point of origin
		 * (home position).
		 */
 
		if (!costmap2d_local->getRobotPose(robotPose)) {
			ROS_ERROR("Failed to get RobotPose");
		}
		visualize_goal_point(robotPose.getOrigin().getX(),
				robotPose.getOrigin().getY());

		// transmit three times, since rviz need at least 1 to buffer before visualizing the point
		for (int i = 0; i <= 2; i++) {
			visualize_home_point();
		}


		// instantiate the planner
		exploration = new explorationPlanner::ExplorationPlanner(c);
                
		/*
		 * Define the first Goal. This is required to have at least one entry
		 * within the vector. Therefore set it to the home position.
		 */
                               
                robot_home_position_x = robotPose.getOrigin().getX();
                robot_home_position_y = robotPose.getOrigin().getY();
		ROS_INFO("Set home point to (%lf,%lf).",robot_home_position_x,robot_home_position_y);
               
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                
                exploration->storeVisitedFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);		             
                exploration->storeFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);           
               
                exploration->setRobotConfig(robot_id, robot_home_position_x, robot_home_position_y, move_base_frame);
                
		ROS_ERROR("                                             ");
		ROS_ERROR("************* INITIALIZING DONE *************");
        ROS_ERROR("                                             ");
                
	}


	void Explorer::explore(int exploreDistanceFromHome) {
		/*
		 * Sleep is required to get the actual 
		 * costmap updated with obstacle and inflated 
		 * obstacle information. This is rquired for the
		 * first time explore() is called.
		 */
		ROS_DEBUG("Sleeping 5s for costmaps to be updated.");
                geometry_msgs::Twist twi;
                //should be a parameter!! only for testing on Wed/17/9/14
                ros::Publisher twi_publisher = nh.advertise<geometry_msgs::Twist>("/Rosaria/cmd_vel",3);
                twi.angular.z = 0.75;
                twi_publisher.publish(twi);
                ros::Duration(5.0).sleep();
                twi_publisher.publish(twi);
                /*
                 * START TAKING THE TIME DURING EXPLORATION     
                 */
                time_start = ros::Time::now();
                
                
        exploration->exploreDistanceFromHome = exploreDistanceFromHome;
        running = true;
        setExplorationFinished(false);
		while (exploration_finished == false || backToHome) {
                if(Simulation == false) {
                        /*
                Simulation == false; 
                         * *****************************************************
                         * FRONTIER DETERMINATION
                         * *****************************************************
                         */

                            ROS_INFO("****************** EXPLORE ******************");

                            /*
                             * Use mutex to lock the critical section (access to the costmap)
                             * since rosspin tries to update the costmap continuously
                             */
                            costmap_mutex.lock();

                            exploration->transformToOwnCoordinates_frontiers();
                            exploration->transformToOwnCoordinates_visited_frontiers();

//    			    ros::Duration(1.0).sleep();
                            exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global);
                            exploration->findFrontiers();
//                            exploration->printFrontiers();
                          
                            exploration->clearVisitedFrontiers();                       
                            exploration->clearUnreachableFrontiers();
                            exploration->clearSeenFrontiers(costmap2d_global);       

                            costmap_mutex.unlock();

                            exploration->publish_frontier_list();  
                            exploration->publish_visited_frontier_list();  

                            ros::Duration(1).sleep();
                            exploration->publish_frontier_list();  
                            exploration->publish_visited_frontier_list();  

                            if (exploration->frontiers.size() == 0 || backToHome) {
                                navigateHome();
                                exploration_finished = true;
                                backToHome = false;
                                continue;
                            }

                            /*
                             * Sleep to ensure that frontiers are exchanged
                             */
                            ros::Duration(1).sleep();
                           
                            /*
                             * *****************************************************
                             * FRONTIER SELECTION
                             * *****************************************************
                             */
 
                            int retry = 0;
                            bool successful = false;
                            // retry navigation without recalculating frontiers 
                            for (int retry = 0; retry <= 3 || successful == false ;retry++) {
                                successful = navigateToNextFrontier(frontier_selection);
                            }

                            exploration->visualize_Cluster_Cells();
                            exploration->visualize_Frontiers();

    //                            exploration->clearVisitedFrontiers();
    //                            exploration->clearUnreachableFrontiers();  

    //                            exploration->publish_frontier_list();  
    //                            exploration->publish_visited_frontier_list();                           
//                            }
                        }
			else
			{
				ROS_WARN("No navigation performed");				
			}		
			ROS_DEBUG("                                             ");
                        ROS_DEBUG("                                             ");
		}
        // END WHILE
        running = false;
	}         
           

    bool Explorer::navigateToNextFrontier(int frontier_selection) {
        int count = 0;
        bool backoff_sucessfull = false, navigate_to_goal = false;
        bool negotiation;
                            
        std::vector<double> final_goal;
        std::vector<double> backoffGoal;
        std::vector<std::string> robot_str;

        /*********** EXPLORATION STRATEGY ************
         * 0 ... Navigate to nearest frontier TRAVEL PATH
         * 1 ... Navigate using auctioning with cluster selection using 
         *       NEAREST selection (Kuhn-Munkres)
         * 2 ... Navigate to furthest frontier
         * 3 ... Navigate to nearest frontier EUCLIDEAN DISTANCE
         * 4 ... Navigate to random Frontier
         * 5 ... Cluster frontiers, then navigate to nearest cluster 
         *       using EUCLIDEAN DISTANCE (with and without
         *       negotiation)
         * 6 ... Cluster frontiers, then navigate to random cluster
         *       (with and without negotiation)
         */

        /******************** SORT *******************
         * Choose which strategy to take.
         * 1 ... Sort the buffer from furthest to nearest frontier
         * 2 ... Sort the buffer from nearest to furthest frontier, normalized to the
         *       robots actual position (EUCLIDEAN DISTANCE)
         * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
         * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN DISTANCE)
         */
        if(frontier_selection < 0 || frontier_selection > 6)
        {
            ROS_FATAL("You selected an invalid exploration strategy. Please make sure to set it in the interval [0,6]. The current value is %d.", frontier_selection);
        }
        if(frontier_selection == 2)
        {

            exploration->sort(1); 

            /* 
             * Choose which strategy to take.
             * 1 ... least to first goal in list
             * 2 ... first to last goal in list
             *
             * Determine_goal() takes world coordinates stored in
             * the frontiers list(which contain every found frontier with sufficient
             * spacing in between) and returns the most attractive one, based on the
             * above defined strategies.
             */    
            while(true)
            {   
                goal_determined = exploration->determine_goal(1, &final_goal, count, 0, &robot_str);
                if(goal_determined == false)
                {
                    break;
                }
                else
                {   
                    //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                    negotiation = true; 
                    if(negotiation == true)
                    {
                        break;
                    }
                    count++;
                }
            }
        }    
        else if(frontier_selection == 3)
        {                           
            exploration->sort(2);

            while(true)
            {   
                goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                if(goal_determined == false)
                {
                    break;
                }
                else
                {
                    //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                    negotiation = true; 
                    if(negotiation == true)
                    {
                        break;
                    }
                    count++;
                }
            }
        }        
        else if(frontier_selection == 0)
        {
            exploration->sort(2);
            exploration->sort(3);

            while(true)
            {   
                goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);
                if(goal_determined == false)
                {
                    break;
                }
                else
                {                        
                    //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                    negotiation = true;
                    if(negotiation == true)
                    {
                        break;
                    }
                    count++;
                }
            }

        }
        else if(frontier_selection == 4)
        {
            while(true)
            {   
                goal_determined = exploration->determine_goal(3, &final_goal, count, 0, &robot_str);
                ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);
                if(goal_determined == false)
                {
                    break;
                }
                else
                {
                    //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                    negotiation = true; 
                    if(negotiation == true)
                    {
                        break;
                    }
                    count++;
                }
            }

        }
        else if(frontier_selection == 5)
        {                            
            if(cluster_initialize_flag == true)
            {
                exploration->clearVisitedAndSeenFrontiersFromClusters();
            }
            else
            {
                /*
                 * This is necessary for the first run 
                 */
                if(robot_id == 0)
                    ros::Duration(10).sleep();

                exploration->sort(2); 
            }

            exploration->clusterFrontiers();                 
            exploration->sort(4);      
            exploration->sort(5);

//                            exploration->visualizeClustersConsole();

            clusters_available_in_pool.clear();
            while(true)
            {                             
                final_goal.clear();
                goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                if(goal_determined == false)
                {
                    ROS_INFO("Another cluster is not available, no cluster determined");                                    
                    break;
                }
                else
                {
                    if(cluster_initialize_flag == false)
                    {
                        /*
                        *  TODO ... just to reduce the selection of the same 
                        * clusters since no auctioning is implemented jet.
                        */
                       if(robot_id == 1)  
                       {
                           ros::Duration(5).sleep();
                       }

                       cluster_initialize_flag = true;
                    }

                    /*
                     * If negotiation is not needed, simply uncomment
                     * and set the negotiation to TRUE.
                     */
                    negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));
//                                        negotiation = true; 
                    if(negotiation == true)
                    {
                        ROS_DEBUG("Negotiation was successful");
                        cluster_element = final_goal.at(4);
                        counter_waiting_for_clusters = 0;
                        break;
                    }
                    else
                    {
                        cluster_element = final_goal.at(4);
                        counter_waiting_for_clusters++;
                        ROS_WARN("Negotiation was not successful, try next cluster");
                    }
                    count++;
                    /*
                     * In order to make one robot wait until the other finds
                     * another cluster to operate in, one have to fill the clusters_available_in_pool
                     * vector if count is increased at least once which determines
                     * that at least one cluster
                     */
                    clusters_available_in_pool.push_back(1);
                }
            }

        } 
        else if(frontier_selection == 6)
        {                            
            if(cluster_initialize_flag == true)
            {
                exploration->clearVisitedAndSeenFrontiersFromClusters();
            }

            exploration->clusterFrontiers();   
            exploration->sort(6);
            exploration->visualizeClustersConsole();

            if(cluster_initialize_flag == false)
            {
                ros::Duration(2).sleep();
            }
            cluster_initialize_flag = true;

            while(true)
            {             
                goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                if(goal_determined == false)
                {
                    break;
                }
                else
                {   
                    /*
                     * If negotiation is not needed, simply uncomment
                     * and set the negotiation to TRUE.
                     */
                    negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));
//                                        negotiation = true; 

                    if(negotiation == true)
                    {
                        cluster_element = final_goal.at(4);
                        break;
                    }
                    else
                    {
                        cluster_element = int(exploration->clusters.size()*rand()/(RAND_MAX));
                        ROS_INFO("Random cluster_element: %d  from available %lu clusters", cluster_element, exploration->clusters.size());
                    }
                    count++;
                }
            }
        } 
        else if(frontier_selection == 1)
        {     
            costmap_mutex.lock();
            if(cluster_initialize_flag == true)
            {
                 exploration->clearVisitedAndSeenFrontiersFromClusters();
            }
            else
            {                              
                /*
                 * This is only necessary for the first run 
                 */

                exploration->sort(2); 
                cluster_initialize_flag = true;
            }

            exploration->clusterFrontiers();                 
            exploration->sort(4);      
            exploration->sort(5);

            costmap_mutex.unlock();

            exploration->visualizeClustersConsole();

            while(true)
            {                             
                final_goal.clear();
                robot_str.clear();
                goal_determined = exploration->determine_goal(5, &final_goal, 0, cluster_element, &robot_str);
                
                ROS_DEBUG("Cluster element: %d", cluster_element);
                
                int cluster_vector_position = -1;

                if(cluster_element != -1)
                {
                    if(exploration->clusters.size() > 0)
                    {                   
                        for (int i = 0; i < exploration->clusters.size(); i++)
                        {
                            if(exploration->clusters.at(i).id == cluster_element)
                            {
                                if(exploration->clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i; 
                                }
                                break;
                            }
                        }
                    }
                }
                ROS_DEBUG("Cluster vector position: %d", cluster_vector_position);
                
                if(cluster_vector_position >= 0)
                {
                    if(exploration->clusters.at(cluster_vector_position).unreachable_frontier_count >= number_unreachable_frontiers_for_cluster)
                    {
                        goal_determined = false;
                        ROS_WARN("Cluster inoperateable");
                    }else
                    {
                        ROS_WARN("Cluster operateable");
                    }
                }
                
                if(goal_determined == false)
                {  
                    
                    ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

                    final_goal.clear();
                    robot_str.clear();
                    clusters_available_in_pool.clear();

                    bool auctioning = exploration->auctioning(&final_goal, &clusters_available_in_pool, &robot_str);
                    if(auctioning == true)
                    {
                        goal_determined = true; 
                        cluster_element = final_goal.at(4);
                        counter_waiting_for_clusters = 0;
                        break;
                    }
                    else
                    {
                        if(exploration->clusters.size() > 0) //clusters_available_in_pool.size() > 0)
                        {
                            ROS_INFO("No cluster was selected but other robots are operating ... waiting for new clusters");
                            counter_waiting_for_clusters++;
                            break;
                        }else
                        {
                            /*
                             * If NO goals are selected at all, iterate over the global
                             * map to find some goals. 
                             */
                            ROS_WARN("No goals are available at all");
                            cluster_element = -1;
                            break;
                        }
                    }
                }
                else
                {
                    ROS_INFO("Still some goals in current cluster, finish this job first");
                    break;
                }
            }

        } 

          /*
     * *****************************************************
     * FRONTIER COORDINATION
     * *****************************************************
     */

        ROS_INFO("Navigating to Goal");

        if(goal_determined == true)
        {
            if(OPERATE_WITH_GOAL_BACKOFF == true)
            {
                ROS_DEBUG("Doing smartGoalBackoff");
                backoff_sucessfull = exploration->smartGoalBackoff(final_goal.at(0),final_goal.at(1), costmap2d_global, &backoffGoal);   
            }
            else
            {
                backoff_sucessfull = true;
            }
        }

        if(backoff_sucessfull == true)
        {
            if(OPERATE_WITH_GOAL_BACKOFF == true)
            {
                ROS_INFO("Doing navigation to backoff goal");
                navigate_to_goal = navigate(backoffGoal);
            }else
            {
                ROS_INFO("Doing navigation to goal");
                navigate_to_goal = navigate(final_goal);
            }
        }
        else if(backoff_sucessfull == false && goal_determined == false)
        {
            navigate_to_goal = navigate(final_goal);
            goal_determined = false;
        }


        if(navigate_to_goal == true && goal_determined == true) {
            exploration->calculate_travel_path(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);

            ROS_INFO("Storeing visited...");
            exploration->storeVisitedFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3)); 
            ROS_DEBUG("Stored Visited frontier");
           
        } else if(navigate_to_goal == false && goal_determined == true) {
            ROS_INFO("Storeing unreachable...");
            storeUnreachableFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3));                            
            ROS_DEBUG("Stored unreachable frontier");
        } else {
            ROS_INFO("Result: navigate_to_goal: %d, goal_determined: %d", navigate_to_goal, goal_determined);
        }

        return goal_determined;
    }

    void Explorer::frontiers() {
            ros::Rate r(5);
            while(ros::ok())
            {
                
                costmap_mutex.lock();
                
                exploration->clearSeenFrontiers(costmap2d_global);
                exploration->clearVisitedFrontiers();
                exploration->clearUnreachableFrontiers(); 
                
                exploration->publish_frontier_list();  
                exploration->publish_visited_frontier_list();  
                
                costmap_mutex.unlock();
                
                r.sleep();
            }
        }
        
        
        void Explorer::map_info()
        {       
            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << "#time,exploration_travel_path_global,exploration_travel_path_average,global_map_progress,local_map_progress,number_of_completed_auctions, number_of_uncompleted_auctions, frontier_selection_strategy, costmap_size, unreachable_frontiers" << std::endl;
            fs_csv.close();
            
            while(ros::ok() && exploration_finished != true)
            {
                costmap_mutex.lock();

                ros::Duration time = ros::Time::now() - time_start;
                double exploration_time = time.toSec();  

                map_progress.global_freespace = global_costmap_size();
                map_progress.local_freespace = local_costmap_size();
                map_progress.time = exploration_time;               
                map_progress_during_exploration.push_back(map_progress);


                double exploration_travel_path_global = (double)exploration->exploration_travel_path_global * 0.02;
                double exploration_travel_path_average = 0;
                if(counter != 0)
                {
                    exploration_travel_path_average = (exploration->exploration_travel_path_global) / counter;
                }

//                ROS_INFO("global map size: %f   at time: %f", map_progress.global_freespace, map_progress.time);
//                ROS_INFO("local map size : %f   at time: %f", map_progress.local_freespace, map_progress.time);
//                ROS_INFO("travel path glo: %d   at time: %f", exploration_travel_path_global, map_progress.time);
//                ROS_INFO("travel path ave: %d   at time: %f", exploration_travel_path_average, map_progress.time);

                fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);

                fs_csv << map_progress.time << "," << exploration_travel_path_global << "," << exploration_travel_path_average << "," << map_progress.global_freespace << "," << map_progress.local_freespace << "," << global_iterattions <<  "," << exploration->number_of_completed_auctions << "," << exploration->number_of_uncompleted_auctions << "," << frontier_selection << "," <<  costmap_width << "," << exploration->unreachable_frontiers.size() <<  std::endl;
//                fs_csv << "travel_path_global   = " << exploration_travel_path_global << std::endl;
//                fs_csv << "travel_path_average  = " << exploration_travel_path_average << std::endl;             
//                fs_csv << "map_progress_global  = " << map_progress.global_freespace << std::endl;
//                fs_csv << "map_progress_average = " << map_progress.local_freespace << std::endl;
//                fs_csv << "time                 = " << map_progress.time << std::endl;
//                fs_csv << "                       " << std::endl;

                fs_csv.close();

                costmap_mutex.unlock();    

                // call map_merger to log data
                map_merger::LogMaps log;
                log.request.log = 12;    /// request local and global map progress
                ROS_DEBUG("Calling map_merger service logOutput");
                if(!mm_log_client.call(log))
                    ROS_WARN("Could not call map_merger service to store log.");
                ROS_DEBUG("Finished service call.");

                save_progress();
                
                ros::Duration(10.0).sleep();
            }
        }
        
        int Explorer::global_costmap_size()
        {
           occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
           int num_map_cells_ = costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
           int free = 0;
           
           for (unsigned int i = 0; i < num_map_cells_; i++) 
           {
                if ((int) occupancy_grid_global[i] == costmap_2d::FREE_SPACE) 
                {
                     free++;
                }
            }   
           return free;
        }
        
        int Explorer::local_costmap_size()
        {   
            if(OPERATE_ON_GLOBAL_MAP == false)
            {
                occupancy_grid_local = costmap2d_local->getCostmap()->getCharMap();
                int num_map_cells_ = costmap2d_local->getCostmap()->getSizeInCellsX() * costmap2d_local->getCostmap()->getSizeInCellsY();
                int free = 0;

                for (unsigned int i = 0; i < num_map_cells_; i++) 
                {
                     if ((int) occupancy_grid_local[i] == costmap_2d::FREE_SPACE) 
                     {
                          free++;
                     }
                 }   
                return free;
            }else
            {
                occupancy_grid_local = costmap2d_local_size->getCostmap()->getCharMap();
                int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() * costmap2d_local_size->getCostmap()->getSizeInCellsY();
                int free = 0;

                for (unsigned int i = 0; i < num_map_cells_; i++) 
                {
                     if ((int) occupancy_grid_local[i] == costmap_2d::FREE_SPACE) 
                     {
                          free++;
                     }
                 }   
                return free;
            }
        }
        
        void Explorer::initLogPath()
        {
           /*
            *  CREATE LOG PATH
            * Following code enables to write the output to a file
            * which is localized at the log_path
            */
            
            nh.param<std::string>("log_path",log_path,"");           

            std::stringstream robot_number;
            robot_number << robot_id;
            std::string prefix = "/robot_";
            std::string robo_name = prefix.append(robot_number.str());    

            log_path = log_path.append("/explorer");
            log_path = log_path.append(robo_name);
            log_path = log_path.append("/");
            ROS_INFO("Logging files to %s", log_path.c_str());

            boost::filesystem::path boost_log_path(log_path.c_str());
            if(!boost::filesystem::exists(boost_log_path))
                if(!boost::filesystem::create_directories(boost_log_path))
                    // directory not created; check if it is there anyways
                    if(!boost::filesystem::is_directory(boost_log_path))
                        ROS_ERROR("Cannot create directory %s.", log_path.c_str());
                    else
                        ROS_INFO("Successfully created directory %s.", log_path.c_str());
             
        }
        
        void Explorer::save_progress(bool final)
        {
            ros::Duration ros_time = ros::Time::now() - time_start;
            
            double exploration_time = ros_time.toSec();           
            int navigation_goals_required = counter;        
            double exploration_travel_path = (double)exploration->exploration_travel_path_global * costmap_resolution;
            double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;
                    
            double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

            std::string tmp_log;
	    if(!final) 
	    {
            	tmp_log = log_file + std::string(".tmp");
            	fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
	    }
 	    else
	    {
		tmp_log = log_file;
            	fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
	    }

            /*
             *  WRITE LOG FILE
             * Write all the output to the log file
             */
            time_t raw_time;
            struct tm* timeinfo;
            time (&raw_time);
            timeinfo = localtime (&raw_time);

            fs << "[Exploration]" << std::endl;
            fs << "time_file_written    			= " << asctime(timeinfo); // << std::endl;
            fs << "start_time           			= " << time_start << std::endl;
            fs << "end_time             			= " << ros::Time::now() << std::endl;
            fs << "exploration_time    			        = " << exploration_time << std::endl;
            fs << "required_goals       			= " << navigation_goals_required << std::endl;  
            fs << "unreachable_goals                            = " << exploration->unreachable_frontiers.size() << std::endl;
            fs << "travel_path_overall  			= " << exploration_travel_path << std::endl;
            fs << "number_of_completed_auctions                 = " << exploration->number_of_completed_auctions << std::endl;
            fs << "number_of_uncompleted_auctions               = " << exploration->number_of_uncompleted_auctions << std::endl;
            fs << "frontier_selection_strategy                  = " << frontier_selection << std::endl;
            fs << "costmap_size                                 = " << costmap_width << std::endl;
            fs << "global costmap iterations                    = " << global_costmap_iteration << std::endl;
            
	    double param_double;
        int param_int;
	    std::string param;
        /*param = robot_prefix + "/explorer/local_costmap/height";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/height",param_int,-1);
        fs << "explorer_local_costmap_height 		= " << param_int << std::endl;

        /*param = robot_prefix + "/explorer/local_costmap/width";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/width",param_int,-1);
        fs << "explorer_local_costmap_width 		= " << param_int << std::endl;

	    param = robot_prefix + "/move_base/local_costmap/height";
            ros::param::get(param,param_double);
	    fs << "move_base_local_costmap_height 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/local_costmap/width";
            ros::param::get(param,param_double);
	    fs << "move_base_local_costmap_width 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/global_costmap/obstacle_layer/raytrace_range";
            ros::param::get(param,param_double);
	    fs << "move_base_raytrace_range 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/global_costmap/obstacle_layer/obstacle_range";
            ros::param::get(param,param_double);
	    fs << "move_base_obstacle_range 		= " << param_double << std::endl;

//	    param = robot_prefix + "/navigation/global_costmap/obstacle_layer/raytrace_range";
        nh.getParam("/global_costmap/obstacle_layer/raytrace_range",param_double);
        ros::param::get(param,param_double);
	    fs << "explorer_raytrace_range 		= " << param_double << std::endl;

        //param = robot_prefix + "/navigation/global_costmap/obstacle_layer/obstacle_range";
        //    ros::param::get(param,param_double);
        nh.getParam("/global_costmap/obstacle_layer/obstacle_range",param_double);
	    fs << "explorer_obstacle_range 		= " << param_double << std::endl;

	    if(final)
            	fs << "complete             			= " << "1" << std::endl;
	    else
            	fs << "complete             			= " << "0" << std::endl;
            
            fs.close();
//            ROS_INFO("Wrote file %s\n", log_file.c_str());                    
            
            
            /*
             * Inform map_merger to save maps
             */
            
           if(final)
	    {
		    map_merger::LogMaps log;
		    log.request.log = 3;    /// request local and global map
		    if(!mm_log_client.call(log))
			ROS_WARN("Could not call map_merger service to store log.");
	    }
    
        }
        
        void Explorer::exploration_has_finished()
        {
            ros::Duration ros_time = ros::Time::now() - time_start;
            
            double exploration_time = ros_time.toSec();           
            int navigation_goals_required = counter;        
            double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
            double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;
            ROS_INFO("overall freespace in the global map: %f", size_global_map);
            
            for(int i = 0; i< map_progress_during_exploration.size(); i++)
            {
                ROS_INFO("map progress: %f",(map_progress_during_exploration.at(i).global_freespace/size_global_map)*100);
            }
           
            ROS_DEBUG("******************************************");
            ROS_DEBUG("******************************************");
            ROS_DEBUG("TIME: %f sec  GOALS: %d  PATH: %f  COMPLETED AUCTIONS: %d  UNCOMPLETED AUCTIONS: %d", exploration_time, navigation_goals_required, exploration_travel_path, exploration->number_of_completed_auctions, exploration->number_of_uncompleted_auctions);
            ROS_DEBUG("******************************************");
            ROS_DEBUG("******************************************");
            
                  
            double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);
                    
            /*
             *  WRITE LOG FILE
             * Write all the output to the log file
             */
           save_progress(true);
            ROS_INFO("Wrote file %s\n", log_file.c_str());                    
            
            
            /*
             * Inform map_merger to save maps
             */
            
            map_merger::LogMaps log;
            log.request.log = 3;    /// request local and global map
            if(!mm_log_client.call(log))
                ROS_WARN("Could not call map_merger service to store log.");

                
            // Indicte end of simulation for this robot
            this->indicateSimulationEnd();
   
//#ifdef PROFILE
//HeapProfilerStop();
//ProfilerStop();
//#endif
        }
        
        void Explorer::indicateSimulationEnd()
        {
            /// FIXME: remove this stuff once ported to multicast
            std::stringstream robot_number;
            robot_number << robot_id;
            
            std::string prefix = "/robot_";
            std::string status_directory = "/simulation_status";
            std::string robo_name = prefix.append(robot_number.str());    
            std::string file_suffix(".finished");

            std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
            std::string status_path = ros_package_path + status_directory;
            std::string status_file = status_path + robo_name + file_suffix;

            /// TODO: check whether directory exists
            boost::filesystem::path boost_status_path(status_path.c_str());
            if(!boost::filesystem::exists(boost_status_path))
                if(!boost::filesystem::create_directories(boost_status_path))
                    ROS_ERROR("Cannot create directory %s.", status_path.c_str());
            std::ofstream outfile(status_file.c_str());
            outfile.close();
            ROS_INFO("Creating file %s to indicate end of exploration.", status_file.c_str());


        }
        
        
	bool Explorer::iterate_global_costmap(std::vector<double> *global_goal, std::vector<std::string> *robot_str)
	{
            global_costmap_iteration++;
            int counter = 0;
            bool exploration_flag;
            
            costmap_mutex.lock();

            exploration->transformToOwnCoordinates_frontiers();
            exploration->transformToOwnCoordinates_visited_frontiers();

//    			    ros::Duration(1.0).sleep();
            exploration->initialize_planner("exploration planner", costmap2d_global, costmap2d_global);
            exploration->findFrontiers();
//            exploration->printFrontiers();

            exploration->clearVisitedFrontiers();                       
            exploration->clearUnreachableFrontiers();
            exploration->clearSeenFrontiers(costmap2d_global);       

           costmap_mutex.unlock();
                            
           exploration->visualize_Frontiers();
           
                if(frontier_selection < 5 && frontier_selection != 1)
                {
                   exploration->sort(2);
                   
                    while(true)
                    {   
                        exploration_flag = exploration->determine_goal(2, global_goal, counter, -1, robot_str);
                        
                       
                        if(exploration_flag == false)
                        {
                           break;
                        }
                        else
                        {    
                            bool negotiation = true; 
                            negotiation = exploration->negotiate_Frontier(global_goal->at(0),global_goal->at(1),global_goal->at(2),global_goal->at(3), -1);
                            
                            if(negotiation == true)
                            {     
                                return true;
                                break;
                            }           
                            counter++;
                        }
                    }    
                }else if(frontier_selection == 1 || frontier_selection == 6 || frontier_selection == 5)
                {
                    costmap_mutex.lock();
                    exploration->clearVisitedAndSeenFrontiersFromClusters();
    
                    exploration->clusterFrontiers(); 
                    
                    exploration->sort(4);      
                    exploration->sort(5);

                    costmap_mutex.unlock(); 
                    
//                    exploration->visualizeClustersConsole();

                    cluster_element = -1;
                    
                    while(true)
                    {                             
                        std::vector<double> goal_vector;
                        std::vector<std::string> robot_str_name;
                        std::vector<int> clusters_used_by_others; 
                        
                        goal_determined = exploration->determine_goal(5, &goal_vector, 0, cluster_element, robot_str);

                        if(goal_determined == false)
                        {  
                            ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

                            goal_vector.clear();
                            bool auctioning = exploration->auctioning(&goal_vector, &clusters_used_by_others, &robot_str_name);
                            if(auctioning == true)
                            {                               
                                goal_determined = true; 
                                cluster_element = goal_vector.at(4);
                                
                                global_goal->push_back(goal_vector.at(0));
                                global_goal->push_back(goal_vector.at(1));
                                global_goal->push_back(goal_vector.at(2));
                                global_goal->push_back(goal_vector.at(3));
                                global_goal->push_back(goal_vector.at(4));
                                
                                robot_str->push_back(robot_str_name.at(0));
                                return true; 
                            }
                            else
                            {
                                /*
                                 * If NO goals are selected at all, iterate over the global
                                 * map to find some goals. 
                                 */
                                ROS_WARN("No goals are available at all");
                                cluster_element = -1;
                                break;
                            }
                        }
                        else
                        {
                            ROS_INFO("Still some goals in current cluster, finish this job first");
                            break;
                        }
                    }
//                    while(true)
//                    {                            
//                        std::vector<double> goal_vector;
//                        goal_determined = exploration->determine_goal(4, &goal_vector, counter, cluster_element);
//
//                        if(goal_determined == false)
//                        {
//                            ROS_INFO("Another cluster is not available, no cluster determined");
//                            return false;
//                        }
//                        else
//                        {
//                             /*
//                             * If negotiation is not needed, simply uncomment
//                             * and set the negotiation to TRUE.
//                             */
//                            bool negotiation = exploration->negotiate_Frontier(goal_vector.at(0),goal_vector.at(1),goal_vector.at(2),goal_vector.at(3),goal_vector.at(4));
//                            if(negotiation == true)
//                            {
//                                ROS_INFO("Negotiation was successful");
//                                cluster_element = goal_vector.at(4);
//
//                                global_goal->push_back(goal_vector.at(0));
//                                global_goal->push_back(goal_vector.at(1));
//                                global_goal->push_back(goal_vector.at(2));
//                                global_goal->push_back(goal_vector.at(3));
//                                global_goal->push_back(goal_vector.at(4));
//                                                              
//                                return true;
//                            }
//                            else
//                            {              
//                                cluster_element = goal_vector.at(4);
//                                ROS_ERROR("Negotiation was not successful, try next cluster");
//                            }
//                            counter++;
//                        }
//                    }  
                    exploration->visualize_Cluster_Cells();
            }
              
//            exploration->visualize_Clusters();
//            exploration->visualize_visited_Frontiers();

            global_iterattions++;
            return false;              
	}

        
	bool Explorer::navigate(std::vector<double> goal) {

		/*
		 * If received goal is not empty (x=0 y=0), drive the robot to this point
		 * and mark that goal as seen in the last_goal_position vector!!!
		 * Otherwise turn the robot by 90° to the right and search again for a better
		 * frontier.
		 */
                bool completed_navigation = false;
		if (goal_determined == true)
		{
                        visualize_goal_point(goal.at(0), goal.at(1));
			
                        counter++;
			ROS_INFO("GOAL %d:  x: %f      y: %f", counter, goal.at(0), goal.at(1));
        
			completed_navigation = move_robot(counter, goal.at(0), goal.at(1));
			rotation_counter = 0;
		}
		else
		{
			rotation_counter++;
//                        ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", clusters_available_in_pool.size(), counter_waiting_for_clusters);
			 ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", exploration->clusters.size(), counter_waiting_for_clusters);			
//                        if(clusters_available_in_pool.size() <= 0 || counter_waiting_for_clusters > 10) //(rotation_counter >= 2)
			if(exploration->clusters.size() == 0 || counter_waiting_for_clusters > 10) //(rotation_counter >= 2)
                        {
                            ROS_INFO("Iterating over GLOBAL COSTMAP to find a goal!!!!");
                            std::vector<double> global_goal;
                            std::vector<std::string> robot_str;
                            bool global_costmap_goal = iterate_global_costmap(&global_goal, &robot_str);
                                 
                            if(global_costmap_goal == false)
                            {
//                                global_iterations_counter++;
//                                if(global_iterations_counter >= 5)
//                                {
                                    completed_navigation = navigateHome();
                                    
                                    exploration_finished = true;
//                                }else
//                                {
//                                    ROS_INFO("No Goal determined on GLOBAL COSTMAP for the %d time. Still wait ...", global_iterations_counter);
//                                }
                            }else
                            {
                                    counter++;
                                    ROS_INFO("GOAL %d:  x: %f      y: %f", counter, global_goal.at(0), global_goal.at(1));
                                    std::vector<double> backoffGoal;
                                    bool backoff_sucessfull = exploration->smartGoalBackoff(global_goal.at(0),global_goal.at(1), costmap2d_global, &backoffGoal);
                                    
                                    if(backoff_sucessfull == true)
                                    {
                                        ROS_DEBUG("doing navigation to back-off goal");
                                        visualize_goal_point(backoffGoal.at(0), backoffGoal.at(1));

                                        completed_navigation = move_robot(counter, backoffGoal.at(0), backoffGoal.at(1));
                                        rotation_counter = 0;
                                        if(completed_navigation == true)
                                        {
                                            exploration->calculate_travel_path(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);
                                
                                            ROS_INFO("Storing visited...");
                                            exploration->storeVisitedFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3)); 
                                            ROS_INFO("Stored Visited frontier");
                                        }   
                                        else
                                        {
                                            ROS_INFO("Storing unreachable...");
                                            exploration->storeUnreachableFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3));
                                            ROS_INFO("Stored unreachable frontier");
                                        }                          
                                    }
                                    else if(backoff_sucessfull == false)
                                    {
                                        ROS_WARN("Navigation to global costmap back-off goal not possible"); 
                                        ROS_INFO("Storing as unreachable...");
                                        exploration->storeUnreachableFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2), robot_str.at(0), global_goal.at(3));
                                        ROS_INFO("Stored unreachable frontier");
                                    }
                            }
			}else
			{
				counter++;
                                ROS_INFO("GOAL %d:  rotation", counter);
				completed_navigation = turn_robot(counter);
			}
		}      
                return(completed_navigation);
	}

    bool Explorer::navigateHome() {
        counter++;
        ROS_INFO("GOAL %d: BACK TO HOME   x: %f    y: %f", counter, home_point_x, home_point_y);
                                    
        /*
         * If the robot should drive to the home position
         * exploration_has_finished() has to be uncommented. 
         * It creates a file which is the immediate trigger to 
         * shut down the ros_node. Therefore the navigation process
         * is canceled . 
         */
        exploration_has_finished();
        visualize_goal_point(home_point_x, home_point_y);
        bool completed_navigation = false;
        for(int i = 0; i< 5; i++){
            if(completed_navigation == false) {
                // reset finish flag to prevent the move_robot method to cancel the goal
                exploration_finished = false;
                completed_navigation = move_robot(counter, home_point_x, home_point_y); 
                exploration_finished = true;
            } else {
                break;
            }                                       
        }
        return completed_navigation;
    }

	void Explorer::visualize_goal_point(double x, double y) {

		goalPoint.header.seq = goal_point_message++;
		goalPoint.header.stamp = ros::Time::now();
                goalPoint.header.frame_id = move_base_frame; //"map"
		goalPoint.point.x = x;// - robotPose.getOrigin().getX();
		goalPoint.point.y = y;// - robotPose.getOrigin().getY();

		ros::NodeHandle nh_Point("goalPoint");
		pub_Point = nh_Point.advertise < geometry_msgs::PointStamped
				> ("goalPoint", 100, true);
		pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);
	}

	void Explorer::visualize_home_point() {

		homePoint.header.seq = home_point_message++;
		homePoint.header.stamp = ros::Time::now();
                homePoint.header.frame_id = move_base_frame; //"map";
		home_point_x = robotPose.getOrigin().getX();
		home_point_y = robotPose.getOrigin().getY();
		homePoint.point.x = home_point_x;
		homePoint.point.y = home_point_y;

		ros::NodeHandle nh("homePoint");
		pub_home_Point = nh.advertise < geometry_msgs::PointStamped
				> ("homePoint", 100, true);
		pub_home_Point.publish < geometry_msgs::PointStamped > (homePoint);

	}

	bool Explorer::move_robot(int seq, double position_x, double position_y) {

        int distance = calculatePlanDistance(0, 0, true, position_x, position_y);
        if ((distance > exploration->exploreDistanceFromHome
            && exploration->exploreDistanceFromHome > 0)
            || distance > 10000) {
            ROS_ERROR("cancel movement: travel distance to far");
            return false;
        }

        /*try {
            // check if there is a valid plan for this position
            if (!checkIfFrontierIsReachable(position_x, position_y)) {
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                return false;
            }
        } catch(...) {
            ROS_ERROR("ERROR checking reachablility");
        }*/

            exploration->next_auction_position_x = position_x;
            exploration->next_auction_position_y = position_y;
            
		/*
		 * Move the robot with the help of an action client. Goal positions are
		 * transmiftted to the robot and feedback is given about the actual
		 * driving state of the robot.
		 */
        if (!costmap2d_local->getRobotPose(robotPose)) {
			ROS_ERROR("Failed to get RobotPose");
		}
                
		actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
				> ac("move_base", true);

                while (!ac.waitForServer(ros::Duration(10.0)))
			;
           
		move_base_msgs::MoveBaseGoal goal_msgs;

		goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
//		goal_msgs.target_pose.header.stamp = ros::Time::now();
                goal_msgs.target_pose.header.frame_id = move_base_frame; //"map";
		goal_msgs.target_pose.pose.position.x = position_x;// - robotPose.getOrigin().getX(); //goals[0].pose.position.x;
		goal_msgs.target_pose.pose.position.y = position_y;// - robotPose.getOrigin().getY();
		goal_msgs.target_pose.pose.position.z = 0;
		goal_msgs.target_pose.pose.orientation.x = 0; //sin(angle/2); // goals[0].pose.orientation.x;
		goal_msgs.target_pose.pose.orientation.y = 0;
		goal_msgs.target_pose.pose.orientation.z = 0;
		goal_msgs.target_pose.pose.orientation.w = 1;

		ac.sendGoal(goal_msgs);
         
        int recalculateCounter = 0;

        //ac.waitForResult(ros::Duration(20)); EDIT Peter: Test if it also works with smaller value!
        ac.waitForResult(ros::Duration(waitForResult)); //here Parameter!
        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING
            || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {

            if (exploration_finished) {
                // cancel navigation
                ac.cancelGoal();
                return true;
            }

            recalculateCounter++;
            if (recalculateCounter == 30) {
                recalculateCounter = 0;
                // move_base can not calculate plan while running
                ac.cancelGoal();

                while(ac.getState() == actionlib::SimpleClientGoalState::PENDING
                    || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                    ros::Duration(0.5).sleep();
                }
                if (calculatePlanDistance(home_point_x, home_point_y, true, position_x, position_y) > exploration->exploreDistanceFromHome) {
                    ROS_ERROR("cancel movement: travel distance to far");
                    return false;
                }
                ac.sendGoal(goal_msgs);
            }
            ros::Duration(1).sleep();
        }
		ROS_INFO("Not longer PENDING");

	   /*while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            if (exploration_finished) {
                // cancel navigation
                ac.cancelGoal();
                return true;
            }

            recalculateCounter++;
            if (recalculateCounter % 10 == 0) {
                // move_base can not calculate plan while running
                ac.cancelGoal();
                if (calculatePlanDistance(home_point_x, home_point_y, position_x, position_y) > exploration->exploreDistanceFromHome) {
                    return false;
                }
                ac.sendGoal(goal_msgs);
            }

            ros::Duration(1).sleep();
        }*/
		ROS_INFO("Not longer ACTIVE");

		while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
			if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
				ROS_INFO("ABORTED");
                                
                                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                                exploration->next_auction_position_y = robotPose.getOrigin().getY();
				return false;
			}
			/*
			 double my_time = ros::Time::now().toSec();
			 //ROS_ERROR("my_time: %f     timeout: %f",my_time,timeout);
			 if(my_time >= timeout)
			 {
			 ROS_ERROR("Timeout exceeded");
			 break;
			 }
			 if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){ROS_ERROR("PREEMPTED");}
			 if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){ROS_ERROR("ABORTED");}
			 if(ac.getState() == actionlib::SimpleClientGoalState::REJECTED){ROS_ERROR("REJECTED");}
			 if(ac.getState() == actionlib::SimpleClientGoalState::RECALLED){ROS_ERROR("RECALLED");}
			 */
		}
                ROS_INFO("TARGET REACHED");
                
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
		return true;
	}

    void Explorer::setExplorationFinished(bool b) {
        exploration_finished = b;
        exploration->exploration_finished = b;
    }

	bool Explorer::turn_robot(int seq) {
		
                double angle = 45;

        if (!costmap2d_local->getRobotPose(robotPose)) {
			ROS_ERROR("Failed to get RobotPose");
		}
                
		actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
				> ac("move_base", true);
		while (!ac.waitForServer(ros::Duration(10.0)))
			;

		move_base_msgs::MoveBaseGoal goal_msgs;

		goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
		goal_msgs.target_pose.header.stamp = ros::Time::now();

                goal_msgs.target_pose.header.frame_id = move_base_frame;//"map";
		goal_msgs.target_pose.pose.position.x = robotPose.getOrigin().getX();
		goal_msgs.target_pose.pose.position.y = robotPose.getOrigin().getY();
		goal_msgs.target_pose.pose.position.z = 0;
		goal_msgs.target_pose.pose.orientation.x = 0; //sin(angle/2); // goals[0].pose.orientation.x;
		goal_msgs.target_pose.pose.orientation.y = 0;
		goal_msgs.target_pose.pose.orientation.z = sin(angle / 2);
		goal_msgs.target_pose.pose.orientation.w = cos(angle / 2);

		ac.sendGoal(goal_msgs);
		ac.waitForResult(ros::Duration(waitForResult));

		while (ac.getState() == actionlib::SimpleClientGoalState::PENDING) {
            if (exploration_finished) {
                ac.cancelGoal();
                return true;
            }
            ros::Duration(0.5).sleep();
        }
		ROS_INFO("Not longer PENDING");

		while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            if (exploration_finished) {
                ac.cancelGoal();
                return true;
            }
            ros::Duration(0.5).sleep();
        }
		ROS_INFO("Not longer ACTIVE");

		while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
			if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
				ROS_INFO("ABORTED");                                
				return false;
			}
		}
		ROS_INFO("ROTATION ACCOMBLISHED");
		return true;
	}

	void Explorer::feedbackCallback(
			const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
		feedback_value = msg.get()->status.status;
		feedback_succeed_value =
				msg.get()->feedback.base_position.pose.orientation.w;

	}

	bool Explorer::target_reached(void) {

		ros::NodeHandle nh_sub_move_base;
		sub_move_base = nh_sub_move_base.subscribe("feedback", 1000,
				&Explorer::feedbackCallback, this);

		while (feedback_value != feedback_succeed_value)
			;

		return true;
	}

    void Explorer::storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id) {
        exploration->storeUnreachableFrontier(x, y, detected_by_robot, detected_by_robot_str, id);
/*        try {
            for (int i = 0; i < exploration->frontiers.size(); i++) {
                double diff_x = fabs(x - exploration->frontiers.at(i).x_coordinate);
                double diff_y = fabs(y - exploration->frontiers.at(i).y_coordinate);


                // TODO improve
                if (diff_x > 0.5 && diff_x <= 10 && diff_y > 0.5 && diff_y <= 10) {

                    /*if (calculatePlanDistance(home_point_x, home_point_y, true, exploration->frontiers.at(i).x_coordinate, exploration->frontiers.at(i).y_coordinate) > exploration->exploreDistanceFromHome
                        && exploration->exploreDistanceFromHome > 0) {
                        exploration->storeUnreachableFrontier(exploration->frontiers.at(i).x_coordinate,
                                exploration->frontiers.at(i).y_coordinate,
                                detected_by_robot,
                                detected_by_robot_str,
                                exploration->frontiers.at(i).id
                        );
                        continue;
                    }

                    geometry_msgs::PoseStamped start;
                    start.header.seq = home_point_message++;
                    start.header.stamp = ros::Time::now();
                    start.header.frame_id = move_base_frame;
                    start.pose.position.x = exploration->frontiers.at(i).x_coordinate;
                    start.pose.position.y = exploration->frontiers.at(i).y_coordinate;
                    nav_msgs::GetPlan plan;
                    plan.request.goal = start;
                    if (ros::service::call("move_base/make_plan", plan)) {
                        ROS_ERROR("Called Service: make_plan");
                        int size = plan.response.plan.poses.size();
                        if (size == 0) {
                            ROS_ERROR("plan is empty"); 
                            exploration->storeUnreachableFrontier(exploration->frontiers.at(i).x_coordinate,
                                exploration->frontiers.at(i).y_coordinate,
                                detected_by_robot,
                                detected_by_robot_str,
                                exploration->frontiers.at(i).id
                            );
            
                        }
                    }            
                }
            }
        } catch(...){
            ROS_ERROR("ERROR while checking plan");
        }*/
    }

    int Explorer::calculatePlanDistance(double startX, double startY, bool fromCurrentPlace, double goalX, double goalY) {

        ROS_INFO("[Explorer] calculatePlanDistance");
        nav_msgs::GetPlan plan;


        if (fromCurrentPlace == false) {
            geometry_msgs::PoseStamped start;
            start.header.seq = home_point_message++;
            start.header.stamp = ros::Time::now();
            start.header.frame_id = move_base_frame;
            start.pose.position.x = startX;
            start.pose.position.y = startY;
            plan.request.start = start;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.seq = home_point_message++;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = move_base_frame;
        goal.pose.position.x = goalX;
        goal.pose.position.y = goalY;
        plan.request.goal = goal;
        if (ros::service::call("move_base/make_plan", plan)) {
            ROS_ERROR("Called Service: make_plan");
            int size = plan.response.plan.poses.size();

            double distance = -1;

            float x = startX;
            float y = startY;

            float xUnknown = startX;
            float yUnknown = startY;
            double distanceUnknown = -1;

            int unknownCounter = 0;
            for (int i=0; i<size; i++) {
                
                if (distance < 0) {
                    distance = 0;
                    x = plan.response.plan.poses[i].pose.position.x;
                    y = plan.response.plan.poses[i].pose.position.y;
                    continue;
                }

                float x2 = plan.response.plan.poses[i].pose.position.x;
                float y2 = plan.response.plan.poses[i].pose.position.y;

                // check the point in the middle to avoid error when a line crosses unknown space
                float middleX = (x + x2) / 2;
                float middleY = (y + y2) / 2;
                bool unknownSpace = exploration->isWorldPointInUnknownSpace(middleX, middleY);

                if (unknownSpace == true) {
                    ROS_ERROR_STREAM("Middle in unknown Space x " << middleX << "y: " << middleY);
                    unknownSpace = false;
                }

                double diff_x = x - x2;
                double diff_y = y - y2;
                x = x2;
                y = y2;
                double distance2 = sqrt( (diff_x * diff_x) + (diff_y * diff_y) );

                distance = distance + distance2;

                unknownSpace = unknownSpace || exploration->isWorldPointInUnknownSpace(x2, y2);
                if ((unknownSpace == true) || distanceUnknown > 0) {
                    //ROS_ERROR("Plan is in unkown space!!! x: %f, y: %f, distance: %f", x2, y2, distanceUnknown);
                    if (distanceUnknown == -1) {
                        distanceUnknown = 0;
                    } else {
                        diff_x = xUnknown - x2;
                        diff_y = yUnknown - y2;
                        distance2 = sqrt( (diff_x * diff_x) + (diff_y * diff_y) );
                        distanceUnknown += distance2;
                    }
                    yUnknown = y2;
                    xUnknown = x2;

                    if (distanceUnknown > 1) {
                        ROS_ERROR("!!!Plan is to long in unkonwn space!!! %f", distanceUnknown);
                        return 99999;
                    }/*
                    unknownCounter++;
                    if (unknownCounter > 10) {
                        return 99999;
                    }*/
                    if (unknownSpace == false) {
                        unknownCounter = 0;
                        distanceUnknown = -1;
                    }
                } else {
                    unknownCounter = 0;
                    distanceUnknown = -1;
                }

            }

            ROS_ERROR("[Explorer] calculated Distance: %f", distance);
            return 0; //(int) distance;
        }            
        return 999999;
    }

    bool Explorer::checkIfFrontierIsReachable(double x, double y) {
        geometry_msgs::PoseStamped start;
        start.header.seq = home_point_message++;
        start.header.stamp = ros::Time::now();
        start.header.frame_id = move_base_frame;
        start.pose.position.x = x;
        start.pose.position.y = y;
        nav_msgs::GetPlan plan;
        plan.request.goal = start;
        if (ros::service::call("move_base/make_plan", plan)) {
            ROS_ERROR("Called Service: make_plan");
            int size = plan.response.plan.poses.size();
            if (size == 0) {
                ROS_ERROR("plan is empty"); 
                /*exploration->storeUnreachableFrontier(exploration->frontiers.at(i).x_coordinate,
                    exploration->frontiers.at(i).y_coordinate,
                    detected_by_robot,
                    detected_by_robot_str,
                    exploration->frontiers.at(i).id
                );*/
                return false;
            } else {
                return true;
            }
        } else {
            return true;
        }
    }
            
int main(int argc, char **argv) {
	/*
	 * ROS::init() function needs argc and argv to perform
	 * any argument and remapping that is provided by the
	 * command line. The third argument is the name of the node
	 */
	ros::init(argc, argv, "simple_navigation");
	/*
	 * Create instance of Simple Navigation
	 */
	tf::TransformListener tf(ros::Duration(10));
    config::Config c;
	explorer::Explorer simple(c, tf);

    explorationController::ExplorationController controller(c, simple);

	/*
	 * The ros::spin command is needed to wait for any call-back. This could for
	 * example be a subscription on another topic. Do this to be able to receive a
	 * message.
	 */

	//boost::thread thr_explore(boost::bind(&explorer::Explorer::explore, &simple));	
//        boost::thread thr_frontiers(boost::bind(&SimpleNavigation::frontiers, &simple));
        /*
         * The following thread is only necessary to log simulation results.
         * Otherwise it produces unused output.
         */
        boost::thread thr_map(boost::bind(&explorer::Explorer::map_info, &simple));

        /*
         * FIXME
         * Which rate is required in order not to oversee
         * any callback data (frontiers, negotiation ...)
         */
//        ros::Rate r(5); // FIXME 30
	while (ros::ok()) {
           
            costmap_mutex.lock(); 
            ros::spinOnce();
            costmap_mutex.unlock();
           
//            r.sleep();
            ros::Duration(0.1).sleep();
	}

	//thr_explore.interrupt();
        thr_map.interrupt();
//	thr_frontiers.interrupt();
        
    //    thr_explore.join();
        thr_map.join();
//        thr_frontiers.interrupt();
        
	return 0;
}
