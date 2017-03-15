#include <ros/ros.h>
#include <omnicopter_sim/PosSp.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

ros::Publisher force_pub;
Eigen::Matrix3d R;

void setpointCallback(const omnicopter_sim::PosSp& input){
	if(input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_FORCE){
		// Transform input from ENU to BODY
	}


}

void imuCallback(const sensor_msgs::Imu& input){
	Eigen::Quaternion q;
	q.x() = input.orientation.x;
	q.y() = input.orientation.y;
	q.z() = input.orientation.z;
	q.w() = input.orientation.w;
	R = q.toRotationMatrix();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh;

	ros::Subscriber pos_sp_sub = nh.subscribe("pos_sp",1,setpointCallback);
	force_pub = nh.advertise<geometry_msgs::Vector3Stamped>("force_sp",0);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);

	ros::spin();
}