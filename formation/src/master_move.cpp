#include "ros/ros.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "formation/Formation.h"
#include <sstream>

#define pi 3.1415

ros::Publisher vel_pub;
double laser_data[360];
double safe_range = 0.8;

void move(double speed, double distance, bool isForward);
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
void rotate(double angular_speed,double goal_angle,bool clockwise);
void avoid();


int main(int argc, char **argv){
	//create "master" node
	ros::init(argc,argv,"master");
	ros::NodeHandle masterNode;
	//publish velocity command (maser:tb3_0)
	vel_pub = masterNode.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel",1000);
	//publish formation command
	ros::Publisher formation_pub = masterNode.advertise<formation::Formation>("/formation",1000);
	//subscribe laser data
	ros::Subscriber laser_sub = masterNode.subscribe("/tb3_0/scan",10,laserCallBack);
	ros::Rate loop_rate(100000);

	while(ros::ok()){
		//call wander() function
		//move(0.5,1.0,1);
	}

	ros::spinOnce();

	return 0;
}

//avoid function
void avoid(){
	                geometry_msgs::Twist vel_msg;


        if(laser_data[0]<safe_range || laser_data[10]<safe_range || laser_data[350]<safe_range){

        //set linear velocity
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        //set angular velocity
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);


                for(int i=2;i<90;i++){
                        if(laser_data[i]>safe_range){
                                rotate(1.0,(i+10)*(pi/180),0);
                                break;
                        }else if(laser_data[360-i]>safe_range){
                                rotate(1.0,(i+10)*(pi/180),1);
                                break;
                        }
                }
        }else{

                vel_msg.linear.x = 0.22;
                vel_pub.publish(vel_msg);
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
                vel_pub.publish(vel_msg);
                double t1 = ros::Time::now().toSec();
                current_angle = angular_speed * (t1-t0);
                ros::spinOnce();
                loop_rate.sleep();
        }while(current_angle<goal_angle);
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
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
		vel_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance < distance);
	vel_msg.linear.x = 0;
	vel_pub.publish(vel_msg);
}

//subscribe callback function
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
