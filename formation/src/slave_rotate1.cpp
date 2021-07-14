#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "formation/Formation.h"

#define pi 3.1415

ros::Publisher slave_vel;
formation::Formation name;

double laser_data[360]; 
double safe_range = 0.8;

// Laser call back function decleration
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
//formation Callback change team shape
void formationCallback(const formation::Formation::ConstPtr& msg);

void rotate(double angular_speed,double goal_angle,bool clockwise);
void avoid();
void move(double speed, double distance, bool isForward);

int main(int argc, char** argv){
	//init formation
	name.name = 0;
	//create follower node
	ros::init(argc,argv,"my_tf_listener");
	ros::NodeHandle followerNode;

	slave_vel = followerNode.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel",10);
	ros::Subscriber laser = followerNode.subscribe("/tb3_1/scan",10,laserCallBack);

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
			//listener.waitForTransform("/tb3_1/base_footprint","/tb3_0/base_footprint",ros::Time(0),ros::Duration(10.0));
			listener.lookupTransform("/tb3_1/base_footprint","/tb3_0/base_footprint",ros::Time(0),transform_sm);
		}catch(tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		} 

		//get master to slave transformation
		try{
			//listener.waitForTransform("/tb3_0/base_footprint","/tb3_1/base_footprint",ros::Time(0),ros::Duration(10.0));

                        listener.lookupTransform("/tb3_0/base_footprint","/tb3_1/base_footprint",ros::Time(0),transform_ms);
                }catch(tf::TransformException& ex){
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                }
		
		avoid(); //avoid function

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
                	vel_msg.linear.x = 0.5 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));
		}else if(name.name == 1){
			//triangle
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y()+0.5,transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.25 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y()+0.5,2));
		}else if(name.name == 2){
			//diamond
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y()+0.5,transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.25 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y()+0.5,2));
		}else{
			//line
			vel_msg.angular.z = 4.0 * atan2(transform_sm.getOrigin().y(),transform_sm.getOrigin().x());
        	        vel_msg.linear.x = 0.3 * sqrt(pow(transform_sm.getOrigin().x(),2) + pow(transform_sm.getOrigin().y(),2));

		}

		slave_vel.publish(vel_msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

//avoid function
void avoid(){

	if(laser_data[0]<safe_range || laser_data[10]<safe_range || laser_data[350]<safe_range){

		for(int i=2;i<90;i++){
			if(laser_data[i]>safe_range){
				rotate(1.0,(i+10)*(pi/180),0);
				break;
			}else if(laser_data[360-i]>safe_range){
				rotate(1.0,(i+10)*(pi/180),1);
				break;
			}
		}
	}	
/**
	if(laser_data[89]<0.2){
		rotate(1.0,90,1);
		move(1.0,0.1,1);		
	}
	if(laser_data[359]<0.2){
		rotate(1.0,90,0);
		move(1.0,0.1,1);

	}
**/
}


//rotate function
void rotate(double angular_speed,double goal_angle,bool clockwise){
	geometry_msgs::Twist vel_msg;
	//set linear velocity
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set angular velocity
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if(clockwise){
		vel_msg.angular.z = -fabs(angular_speed);
	}else{
		vel_msg.angular.z = fabs(angular_speed);
	}

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);

	do{
		slave_vel.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<goal_angle);
	vel_msg.angular.z = 0;
	slave_vel.publish(vel_msg);
}

//control the robot move (go forward for a certain distance)
void move(double speed, double distance, bool isForward){
        geometry_msgs::Twist vel_msg;

        if(isForward)
                vel_msg.linear.x = fabs(speed);
        else
                vel_msg.linear.x = -fabs(speed);

        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;

        double t0 = ros::Time::now().toSec();
        double current_distance = 0.0;
        ros::Rate loop_rate(100);
        do{
                slave_vel.publish(vel_msg);
                double t1 = ros::Time::now().toSec();
                current_distance = speed * (t1-t0);
                ros::spinOnce();
                loop_rate.sleep();
        }while(current_distance < distance);
        vel_msg.linear.x = 0;
        slave_vel.publish(vel_msg);
}


//subscribe call back function
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
	int i;
       for(i=0;i<360;i++){
                laser_data[i] = laser_msg->ranges[i];
		ROS_INFO("laser_data[%d]:%f",i,laser_data[i]);
		ROS_INFO("msg_ranges[%d]:%f",i,laser_msg->ranges[i]);
		if(laser_data[i]<100) ROS_INFO("DENG YU INF");

        }

	/**
	 for(i=0;i<360;i++){
                ROS_INFO("msg_ranges[%d]:%f",i,laser_msg->ranges[i]);
	}
	**/

}

void formationCallback(const formation::Formation::ConstPtr& msg){
	name.name = msg->name;
}
