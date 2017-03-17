#include <ros/ros.h>
#include <omnicopter_sim/PosSp.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

ros::Publisher force_pub;
Eigen::Matrix3d R; // v_body = R*v_enu

void setpointCallback(const omnicopter_sim::PosSp& input){
	// All inputs are given in ENU and the force output is in BODY
	geometry_msgs::Vector3Stamped msg;
	if(input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_FORCE){
		Eigen::Vector3d force_body = R.inverse()*Eigen::Vector3d(input.fx, input.fy, input.fz);
		msg.vector.x = force_body(0);
		msg.vector.y = force_body(1);
		msg.vector.z = force_body(2);
	}
	/*else if (input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_CLIMBRATE_FORCE){
		// If climbrate setpoint is ~0, control altitude

	}
	else if (input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_ALTHOLD_FORCE){
		
	}
	else if (input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_POS){
		// Position P controller and velocity PID+FF
		
	}
	else if (input.type == omnicopter_sim::PosSp::SETPOINT_TYPE_VEL){
		// If velocity setpoint is ~0, control position
		
	}*/
	else {
		ROS_ERROR("Position controller: unknown setpoint type");
		return;
	}
	msg.header.stamp = ros::Time::now();
	force_pub.publish(msg);
}

void imuCallback(const sensor_msgs::Imu& input){
	Eigen::Quaterniond q;
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