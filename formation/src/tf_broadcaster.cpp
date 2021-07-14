#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string robot_name;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char** argv){
	//create broadcaster node
	ros::init(argc,argv,"my_tf_broadcaster");
	if(argc != 2 ){
		ROS_ERROR("need turtle name as argument");
		return -1;
	}
	robot_name = argv[1];

	ros::NodeHandle brNode;
	ros::Subscriber sub = brNode.subscribe(robot_name+"/odom",10,&poseCallback);
	ros::spin();
	return 0;
}

//pose callback and publish tf transform
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,0.0));
	
	/**
       	tf::Quaternion q;
  	//Quaternion to Roll Pitch and Yaw conversion
  	double roll, pitch, yaw;
  	tf::Quaternion quat;
  	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  	q.setRPY(0.0, 0.0, yaw);
  	transform.setRotation(q);
	**/
	transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));

	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",robot_name+"/odom"));
}
