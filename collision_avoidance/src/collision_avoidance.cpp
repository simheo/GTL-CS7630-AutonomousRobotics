#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <utility>
#include <sensor_msgs/Joy.h>
#include <iostream>

class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;
        ros::NodeHandle nh;
		geometry_msgs::Twist res;
        // This might be useful
        double radius; //radius to check
		const static  float minStopDistance = 0.2;
		const static float minAcceptDistance = 0.5;
		float v;
        pcl::PointCloud<pcl::PointXYZ> lastpc;
		//bool stop;
		const static int vmax=1;
        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
            ROS_INFO("published");
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            int d = lastpc[0].x*lastpc[0].x+lastpc[0].y*lastpc[0].y;
            radius =90;
            unsigned int n = lastpc.size();
            ROS_INFO("New point cloud: %d points",n);
            for (int i=0;i<lastpc.size();i++){
				
				if( (abs(lastpc[i].y) < 1) && (lastpc[i].x < 1) && (lastpc[i].x >0)) {
					if((lastpc[i].x*lastpc[i].x+lastpc[i].y*lastpc[i].y) < d){
						d=lastpc[i].x*lastpc[i].x+lastpc[i].y*lastpc[i].y;
						ROS_INFO("%d %.3f %.3f %.3f",i,lastpc[i].x,lastpc[i].y,lastpc[i].z);	
					}
				}
			}	
			if (d < minStopDistance){
				v=0.0;
			}
			else if (d > minAcceptDistance){
				v=(float) vmax;
			}
			else{
				v=vmax*(d-minStopDistance)/(minStopDistance-minAcceptDistance);
			}
        }
        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            if (desired.linear.x > v){
				res.linear.x=v*5;
			}
			else{
				res=desired;
				res.linear.x=res.linear.x*5;
			}
            return res;
            ROS_INFO("modifie coquin");
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this); // analyse du cloudpoint
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this); //analyse de notre vitesse
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1); //modification de notre vitesse au besoin
            nh.param("radius",radius,1.0);
        }

};



int main(int argc, char * argv[]){
    ros::init(argc,argv,"coll_avoid");
    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}
