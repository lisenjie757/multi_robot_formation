#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "formation/Formation.h"

#define pi 3.1415

ros::Publisher slave_vel;
formation::Formation name;


// Laser call back function decleration
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
//formation Callback change team shape
void formationCallback(const formation::Formation::ConstPtr& msg);

int main(int argc, char** argv){
	//init formation
	name.name = 0;
	//create follower node
	ros::init(argc,argv,"my_tf_listener");
	ros::NodeHandle followerNode;

	slave_vel = followerNode.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel",10);
	ros::Subscriber laser = followerNode.subscribe("/tb3_2/scan",1,laserCallBack);

	ros::Subscriber formation = followerNode.subscribe("/formation",10,&formationCallback);

	tf::TransformListener listener;

	ros::Rate rate(1000.0);
	while(followerNode.ok()){
		//slave to master
		tf::StampedTransform transform_sm;
		//master to slave
		tf::StampedTransform transform_ms;

		//get slave to master transformation
		try{
			//listener.waitForTransform("/tb3_2/base_footprint","/tb3_0/base_footprint",ros::Time(0),ros::Duration(10.0));
			listener.lookupTransform("/tb3_2/base_footprint","/tb3_0/base_footprint",ros::Time(0),transform_sm);
		}catch(tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		} 

		//get master to slave transformation
		try{
			//listener.waitForTransform("/tb3_0/base_footprint","/tb3_2/base_footprint",ros::Time(0),ros::Duration(10.0));

                        listener.lookupTransform("/tb3_0/base_footprint","/tb3_2/base_footprint",ros::Time(0),transform_ms);
                }catch(tf::TransformException& ex){
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                }
		
		//print transformation
		ROS_INFO("Master pose w.r.t Slave [x, y]: [%f, %f]", transform_sm.getOrigin().x(), transform_sm.getOrigin().y());
	        ROS_INFO("Orientation: [%f]", atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x()));
    		ROS_INFO("---------------------------------------------");
    		ROS_INFO("Slave pose w.r.t Master [x, y]: [%f, %f]", transform_ms.getOrigin().x(), transform_ms.getOrigin().y());
    		ROS_INFO("Orientation: [%f]", atan2(transform_ms.getOrigin().y(),transform_ms.getOrigin().x()));
		
		//give slave velocity control command
		geometry_msgs::Twist vel_msg;

		if(name.name == 0){
			//line
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x());
                	vel_msg.linear.x = 0.25 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));

		}else if(name.name == 1){
			//triangle
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.25 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));
		}else if(name.name == 2){
			//diamond
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.125 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));
		}else{
			//line
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.25 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));

		}

		slave_vel.publish(vel_msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

//subscribe call back function
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
	ROS_INFO("I am slave tb3_2 in laser call back");
}

void formationCallback(const formation::Formation::ConstPtr& msg){
	name.name = msg->name;
}
