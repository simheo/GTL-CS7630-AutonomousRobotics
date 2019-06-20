#include <stdlib.h> //to pick randomly in set
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include <sstream>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

#define UNKNOWN_SIGNAL -1

class OccupancyGridPlanner {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Subscriber invalid_target_sub;
        ros::Publisher path_pub_;
        ros::Publisher goal_pub_;
        ros::Publisher invalid_target_pub;
        
        ros::Subscriber signal_sub;
        
        //ros::Subscriber end_exploration_sub;
        ////Task Server
        ros::Subscriber expl_trigger_sub;
        
        
        tf::TransformListener listener_;
		
		//Signal Map
        cv::Mat_<float_t> sg_;
		cv::Mat_<cv::Vec3b> sg_rgb_, sg_rgb_marked_;
		cv::Point sg_center_;
		cv::Mat_<uint8_t> sg_counter_;
		cv::Mat_<uint8_t> sg_display_,cropped_sg_;
		
		ros::Time time_last_goal;
		//cv::Point robotPoint;
		//cv::Point start;
		
		geometry_msgs::PoseStamped goal_dock;
		
        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point og_center_;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        double radius_robot;
        unsigned int neighbourhood_;
        bool ready;
        bool debug;
		
		bool expl_trig;
		bool keep_exploring;
		
        typedef std::multimap<float, cv::Point> Heap;
		
		/////////////////////////////////////////
        //// Determine frontier points
        ////////////////////////////////////////
		
        std::vector<cv::Point> frontier_points(cv::Mat_<uint8_t> og,cv::Point start){
			std::vector<cv::Point> border_points;
			
			cv::Size s=og.size();
			
			geometry_msgs::PoseStamped goal;
			
			for (unsigned int j=1;j<s.height-1;j++) {
                for (unsigned int i=1;i<s.width-1;i++) {
					//ROS_INFO("Trying the loop");
					if (og_(j,i) == FREE){
						//ROS_INFO("Found a free point");
						
						//check for neighboors
						for(int deltaj=-1;deltaj<=1;deltaj++){
							for(int deltai=-1;deltai<=1;deltai++){
								if (og_(j+deltaj,i+deltai)==UNKNOWN){
									float distance=cv::norm(cv::Point(i,j)-start);
									//ROS_INFO("Found border points");
									//ROS_INFO("Issue");
									//ROS_INFO("distance %2f",distance);
									if(distance<60.0){
										//ROS_INFO("Found border points");
										border_points.push_back(cv::Point(i,j));
										//ROS_INFO("distance %2f",distance);									
									}
								}
							}
						}				
					}	 
				}
			}
			return border_points;
		}
        
        ///////////////////////////////////////////
        //// Pick goal close to robot randomly
        ///////////////////////////////////////////
        
        geometry_msgs::PoseStamped pick_goal(std::vector<cv::Point> border_points){	
			cv::Point goal_point; 
			int random_pick=rand() % border_points.size(); //pick random index of border_points
			geometry_msgs::PoseStamped goal;
			goal_point=border_points[random_pick];
				
			goal.pose.position.x=((double)(goal_point.x-og_center_.x))*info_.resolution;
			goal.pose.position.y=((double)(goal_point.y-og_center_.y))*info_.resolution;
			goal.pose.position.z=0;
			goal.pose.orientation.x=0;
			goal.pose.orientation.y=0;
			goal.pose.orientation.z=0;
			goal.pose.orientation.w=1;
			
			ros::Duration(0.2).sleep(); //to avoid time problems 
					
			ros::Time t=ros::Time(0);
			goal.header.stamp=t;
			goal.header.frame_id=frame_id_;	
				
			return goal;
		}
        
        /////////////////////////////////
        //// Trigger expl, sent by floor_nav
        /////////////////////////////////
        
		void expl_trig_cb(std_msgs::Bool trig){
			expl_trig=trig.data;
			//ROS_INFO("Trigger Received, let's explore");		
		}
        
        ///////////////////////////////////////////////
        //// Callback in case wrong target sent 
        ///////////////////////////////////////////////
        
        void invalid_target_cb(const std_msgs::String::ConstPtr& msg){
			std::vector<cv::Point> border_points;
            tf::StampedTransform transform;
            cv::Point start;
            start = cv::Point(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution)
                    + og_center_;
 
            
            border_points=frontier_points(og_,start);
            geometry_msgs::PoseStamped goal;
            goal=pick_goal(border_points);
			goal_pub_.publish(goal);
		}
		
		//void end_exploration_cb(geometry_msgs::PoseStamped pose_dock){
		//	goal_dock = pose_dock;
		//	keep_exploring = false;			
		//}
		
        ////////////////////////////////////////////////////
        // Callback for Signal Map 
        ////////////////////////////////////////////////////		
				
		void signal_callback(const std_msgs::Float32 msg){
			//ROS_INFO("In signal_callback");
                        
            //// Create an image to store the value of the grid.
            //sg_ = cv::Mat_<uint8_t>(info_.height, info_.width,0xFF);
            sg_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);

            //// Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=info_.width, 
                         maxy=0, miny=info_.height;
            //ROS_INFO("Initialized everything");
            
            //// Convert the representation into something easy to display.
            for (unsigned int j=0;j<info_.height;j++) {
                for (unsigned int i=0;i<info_.width;i++) {
                    //// Update the bounding box of free or occupied cells.
                    if (sg_(j,i) != UNKNOWN_SIGNAL) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            //ROS_INFO("Finished for loops");
            ////get current position of the robot 
            tf::StampedTransform transform;
			listener_.waitForTransform(frame_id_, base_link_, ros::Time::now(), ros::Duration(1.0));
			listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
			cv::Point curr_pos(int(transform.getOrigin().x() / info_.resolution),
										int(transform.getOrigin().y() / info_.resolution));
			curr_pos += sg_center_;
			//ROS_INFO("Got current pose");

            //if the point was never explored, initialise it
            if (sg_counter_(curr_pos.y, curr_pos.x) == 0) {
				sg_(curr_pos.y, curr_pos.x) = msg.data;
				sg_counter_(curr_pos.y, curr_pos.x)+=1;
				//ROS_INFO("Initialized a point in signal grid");
			}
			//ROS_INFO("I'm after the if");
            ////else compute mean value
            ////else{
				
			
		    ////}

            ////fill the signal grid
            sg_(curr_pos.y, curr_pos.x) = msg.data*sg_counter_(curr_pos.y, curr_pos.x) + sg_(curr_pos.y, curr_pos.x);
			//ROS_INFO("Computed a value of sg");
			sg_counter_(curr_pos.y, curr_pos.x)+=1;
			sg_(curr_pos.y, curr_pos.x) /= (float)sg_counter_(curr_pos.y, curr_pos.x);
			//ROS_INFO("Did the average");
			double sig_value = sg_(curr_pos.y, curr_pos.x);
			sg_display_(curr_pos.y, curr_pos.x) = (uint8_t)(255.0*sig_value);
            //ROS_INFO("Loaded value in sg_display");
            
            ////////////////////////////////////////////
            //// The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            ROS_INFO("Before the conversion");
            //cv::cvtColor(sg_, sg_rgb_, CV_GRAY2RGB);
            ROS_INFO("Did the conversion");
            //// Compute a sub-image that covers only the useful part of the
            //// grid.
            cropped_sg_ = cv::Mat_<uint8_t>(sg_,roi_);
            ROS_INFO("So far so good");
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                //// The occupancy grid is too large to display. We need to scale
                //// it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_sg;
                cv::resize(cropped_sg_,resized_sg,new_size);
                ////cv::imshow( "OccGrid", resized_og );
            } else {
                //// cv::imshow( "OccGrid", cropped_og_ );
                ////cv::imshow( "OccGrid", og_rgb_ );
            }
            
		}
		
		
		
			
        ////////////////////////////////////////////////////
        // Callback for Occupancy Grid
        ////////////////////////////////////////////////////
        
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            
            //ROS_INFO("In og_callback");
                        
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            //og_(j,i)=OCCUPIED;
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            
            int erosion_type=2; //2=MORPH_ELLIPSE
            double erosion_size=radius_robot/info_.resolution;
            
            
            cv::Mat element=getStructuringElement(erosion_type,
												  cv::Size(2*erosion_size+1,2*erosion_size+1),
												  cv::Point(erosion_size,erosion_size));
			erode(og_,og_,element);
			
            if (!ready) {
                //get only here once, so we can initialize everything here
                sg_ = cv::Mat_<float_t>(msg->info.height, msg->info.width,0.0);
                sg_display_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,UNKNOWN_SIGNAL);
                sg_counter_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0.0);
                ready = true;
                for (unsigned int j=0;j<msg->info.height;j++) {
					for (unsigned int i=0;i<msg->info.width;i++) {
						sg_(j,i) = UNKNOWN_SIGNAL;
					}
				}
				for (unsigned int j=0;j<msg->info.height;j++) {
					for (unsigned int i=0;i<msg->info.width;i++) {
						sg_counter_(j,i) = 0;                
					}
				}
				
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            
            //else{ROS_INFO("Didn't receive occupancy grid");}
            
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og;
                cv::resize(cropped_og_,resized_og,new_size);
                //cv::imshow( "OccGrid", resized_og );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                //cv::imshow( "OccGrid", og_rgb_ );
            }
            
            /////////////////////////////////////////////
            //////    Choose goal            
            //////////////////////////////////////////////
            tf::StampedTransform transform;
            cv::Point start;
            start = cv::Point(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution)
                    + og_center_;
 
            
            
            std::vector<cv::Point> border_points;
            border_points=frontier_points(og_,start);
            geometry_msgs::PoseStamped goal;
            goal=pick_goal(border_points);
            
            if (border_points.size()==0){
				ROS_INFO("Exploration is done");			
			}
            
            ros::Time now=ros::Time::now() ;
            
            //if(keep_exploring==true){
			if(now.sec-time_last_goal.sec >=10){
				goal_pub_.publish(goal);
				ROS_INFO("I published a goal");
				time_last_goal=ros::Time::now();
			}
				
			//}
            //else{
			//	ROS_INFO("End exploration, coming home");
			//	goal_pub_.publish(goal_dock	);				
			//}
            
            
            
        }

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }

        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
       
       //////////////////////////////////////////////////////////////////
       ////////////////// TARGET CALLBACK           ////////////////////
       //////////////////////////////////////////////////////////////////
       
        void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
            
            ROS_INFO("In target_callback");
            
            int count=0;
            
            //std_msgs::String ack;
            //std::stringstream ss;
            //ss << "goal received by og planner" << count;
            //ack.data = ss.str();
            
            ////publish acknowledge
            //ack_goal_pub_.publish(ack);
            //ROS_INFO("I received the goal");
            
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped pose;
            if (!ready) {
                ROS_WARN("Ignoring target while the occupancy grid has not been received");
                return;
            }
            ROS_INFO("Received planning request");
            og_rgb_marked_ = og_rgb_.clone();
            // Convert the destination point in the occupancy grid frame. 
            // The debug case is useful is the map is published without
            // gmapping running (for instance with map_server).
            if (debug) {
                pose = *msg;
            } else {
                // This converts target in the grid frame.
                listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                listener_.transformPose(frame_id_,*msg, pose);
                // this gets the current pose in transform
                listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
            }
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
            cv::Point target = cv::Point(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution)
                + og_center_;
            ROS_INFO("Planning target: %.2f %.2f -> %d %d",
                        pose.pose.position.x, pose.pose.position.y, target.x, target.y);
            cv::circle(og_rgb_marked_,target, 20, cv::Scalar(0,0,255));
            cv::imshow( "OccGrid_Marked", og_rgb_marked_ );
            
            // add the frontier point on the og_rgb_marked
            
            
            
            if (!isInGrid(target)) {
                ROS_ERROR("Invalid target point (%.2f %.2f) -> (%d %d)",
                pose.pose.position.x, pose.pose.position.y, target.x, target.y);
                std_msgs::String outside_msg;
				std::stringstream ss;
				ss << "invalid target sent" << count;
				outside_msg.data = ss.str();
                invalid_target_pub.publish(outside_msg);        
                return;
            }
            // Only accept target which are FREE in the grid (HW, Step 5).
            if (og_(target) == OCCUPIED) {
                ROS_ERROR("Invalid target point: occupancy = %d",og_(target));
                ROS_ERROR("Invalid target point (%.2f %.2f) -> (%d %d)",
                pose.pose.position.x, pose.pose.position.y, target.x, target.y);
                std_msgs::String occupied_msg;
				std::stringstream ss;
				ss << "invalid target sent" << count;
				occupied_msg.data = ss.str();
                invalid_target_pub.publish(occupied_msg);
                
                return;
            }

            // Now get the current point in grid coordinates.
            cv::Point start;
            if (debug) {
                start = og_center_;
            } else {
                start = cv::Point(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution)
                    + og_center_;
            }
            ROS_INFO("Planning origin %.2f %.2f -> %d %d",
                    transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
            cv::circle(og_rgb_marked_,start, 10, cv::Scalar(0,255,0));
            cv::imshow( "OccGrid Marked", og_rgb_marked_ );
            if (!isInGrid(start)) {
                ROS_ERROR("Invalid starting point (%.2f %.2f) -> (%d %d)",
                        transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
                return;
            }
            // If the starting point is not FREE there is a bug somewhere, but
            // better to check
            if (og_(start) == OCCUPIED) {
                ROS_ERROR("Invalid start point: occupancy = %d",og_(start));
                return;
            }
            ROS_INFO("Starting planning from (%d, %d) to (%d, %d)",start.x,start.y, target.x, target.y);
            // Here the Dijskstra algorithm starts 
            // The best distance to the goal computed so far. This is
            // initialised with Not-A-Number. 
            cv::Mat_<float> cell_value(og_.size(), NAN);
            // For each cell we need to store a pointer to the coordinates of
            // its best predecessor. 
            cv::Mat_<cv::Vec2s> predecessor(og_.size());

            // The neighbour of a given cell in relative coordinates. The order
            // is important. If we use 4-connexity, then we can use only the
            // first 4 values of the array. If we use 8-connexity we use the
            // full array.
            cv::Point neighbours[8] = {cv::Point(1,0), cv::Point(0,1), cv::Point(-1,0), cv::Point(0, -1),
                cv::Point(1,1), cv::Point(-1,1), cv::Point(-1,-1), cv::Point(1,-1)};
            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.
            float cost[8] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};

            // The core of Dijkstra's Algorithm, a sorted heap, where the first
            // element is always the closer to the start.
            Heap heap;
            heap.insert(Heap::value_type(0, start));
            while (!heap.empty()) {
                // Select the cell at the top of the heap
                Heap::iterator hit = heap.begin();
                // the cell it contains is this_cell
                cv::Point this_cell = hit->second;
                // and its score is this_cost
                float this_cost = hit->first;
                // We can remove it from the heap now.
                heap.erase(hit);
                // Now see where we can go from this_cell
                for (unsigned int i=0;i<neighbourhood_;i++) {
                    cv::Point dest = this_cell + neighbours[i];
                    if (!isInGrid(dest)) {
                        // outside the grid
                        continue;
                    }
                    uint8_t og = og_(dest);
                    if (og == OCCUPIED) {
                        // occupied or unknown
                        continue;
                    }
                    float cv = cell_value(dest);
                    float new_cost = this_cost + cost[i];
                    if (isnan(cv) || (new_cost < cv)) {
                        // found shortest path (or new path), updating the
                        // predecessor and the value of the cell
                        predecessor.at<cv::Vec2s>(dest) = cv::Vec2s(this_cell.x,this_cell.y);
                        cell_value(dest) = new_cost;
                        // And insert the selected cells in the map.
                        heap.insert(Heap::value_type(new_cost,dest));
                    }
                }
            }
            if (isnan(cell_value(target))) {
                // No path found
                std_msgs::String nopath_msg;
				std::stringstream ss;
				ss << "invalid target sent" << count;
				nopath_msg.data = ss.str();
                invalid_target_pub.publish(nopath_msg);                
                
                ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,target.x,target.y);
                
                
                return;
            }
            ROS_INFO("Planning completed");
            // Now extract the path by starting from goal and going through the
            // predecessors until the starting point
            std::list<cv::Point> lpath;
            while (target != start) {
                lpath.push_front(target);
                cv::Vec2s p = predecessor(target);
                target.x = p[0]; target.y = p[1];
            }
            lpath.push_front(start);
            // Finally create a ROS path message
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = frame_id_;
            path.poses.resize(lpath.size());
            std::list<cv::Point>::const_iterator it = lpath.begin();
            unsigned int ipose = 0;
            while (it != lpath.end()) {
                // time stamp is not updated because we're not creating a
                // trajectory at this stage
                path.poses[ipose].header = path.header;
                cv::Point P = *it - og_center_;
                path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                path.poses[ipose].pose.orientation.x = 0;
                path.poses[ipose].pose.orientation.y = 0;
                path.poses[ipose].pose.orientation.z = 0;
                path.poses[ipose].pose.orientation.w = 1;
                ipose++;
                it ++;
            }
            
            //// Task Server
            
            //if(expl_trig==true){
			path_pub_.publish(path);
			ROS_INFO("Request completed");
			//}
            //else{
			//	ROS_INFO("Waiting for Trigger");
			
			//}
        }



    public:
        OccupancyGridPlanner() : nh_("~") {
            int nbour = 4;
            ready = false;
            nh_.param("base_frame",base_link_,std::string("/body"));
            nh_.param("debug",debug,false);
            nh_.param("neighbourhood",nbour,nbour);
            nh_.param("radius_robot",radius_robot,0.2); //former 0.2, increased to 0.4 to avoid problems in replanning
            
            switch (nbour) {
                case 4: neighbourhood_ = nbour; break;
                case 8: neighbourhood_ = nbour; break;
                default: 
                    ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)",nbour);
                    neighbourhood_ = 8;
            }
            og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridPlanner::og_callback,this);
            
            //goal of exploration
            target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::target_callback,this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1,true);
            
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
            invalid_target_pub=nh_.advertise<std_msgs::String>("invalid_target",100,true);
            invalid_target_sub=nh_.subscribe<std_msgs::String>("invalid_target",1,&OccupancyGridPlanner::invalid_target_cb,this);
			
			
			//end_exploration_sub=nh_.subscribe<geometry_msgs::PoseStamped>("end_exploration",1,&OccupancyGridPlanner::end_exploration_cb,this);
			
			//Subscriber to the trigger sent by floor nav
			expl_trigger_sub=nh_.subscribe<std_msgs::Bool>("expl_trigger",1,&OccupancyGridPlanner::expl_trig_cb,this);
			
			//Subscriber to the signal 
			signal_sub=nh_.subscribe("signal",1,&OccupancyGridPlanner::signal_callback,this);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_planner");
    OccupancyGridPlanner ogp;
    cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}
