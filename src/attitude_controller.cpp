#include <ros/ros.h>
#include <omnicopter_sim/AttSp.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Quaternion.h>

#define T_ATT 1.0 // Attitude control time constant
#define T_W 1.0 // Angular rate control time constant

/*
Currently the controller only runs once when a new input is received.
The input persistence of the PWM driver means that the motor outputs corresponding
to this control iteration will be kept until the next iteration.
For safety reasons this may not be optimal, consider making changes.

TODO: fix handling of temporary failsafe (reset setpoint_received, imu_received)
*/

ros::Publisher torque_pub;
const Eigen::Matrix3d J = 0.005*Eigen::Matrix3d::Identity(); //Inertia matrix
bool imu_received = false;
bool setpoint_received = false;
sensor_msgs::Imu imu_data;
omnicopter_sim::AttSp setp;

bool hold_attitude;
geometry_msgs::Quaternion q_hold;

void imuCallback(const sensor_msgs::Imu& input){
	imu_data = input;
	if(!imu_received){
		imu_received = true;
	}
	if(setpoint_received){
		doControl();
	}

}

void setpointCallback(const omnicopter_sim::AttSp& input){
	setp = input;
	if(!setpoint_received){
		setpoint_received = true;
	}
	if(imu_received){
		doControl();
	}
}

void doControl(){
	Eigen::Quaternion q_des;
	Eigen::Quaternion q_curr;
	Eigen::Vector3d w_des;
	Eigen::Vector3d w_curr;

	if(setp.type == omnicopter_sim::AttSp::SETPOINT_TYPE_ATT){
		//Do attitude control
		hold_attitude = false;
		q_des.x() = setp.q.x;
		q_des.y() = setp.q.y;
		q_des.z() = setp.q.z;
		q_des.w() = setp.q.w;
		q_curr.x() = imu_data.orientation.x;
		q_curr.y() = imu_data.orientation.y;
		q_curr.z() = imu_data.orientation.z;
		q_curr.w() = imu_data.orientation.w;

		w_des = control_attitude(q_des, q_curr);
	}
	else if(setp.type == omnicopter_sim::AttSp::SETPOINT_TYPE_RATES){
		if(std::max(setp.wx, setp.wy, setp.wz) < 0.001){
			// If the rate setpoint is ~0, we want to hold the current attitude
			if(!hold_attitude){
				q_hold = setp.q;
				hold_attitude = true;
			}
			q_des.x() = q_hold.q.x;
			q_des.y() = q_hold.q.y;
			q_des.z() = q_hold.q.z;
			q_des.w() = q_hold.q.w;
			q_curr.x() = imu_data.orientation.x;
			q_curr.y() = imu_data.orientation.y;
			q_curr.z() = imu_data.orientation.z;
			q_curr.w() = imu_data.orientation.w;

			w_des = control_attitude(q_des, q_curr);

		}
		else{
			// Do rate control
			hold_attitude = false;
			w_des(0) = setp.wx;
			w_des(1) = setp.wy;
			w_des(2) = setp.wz;
			
		}
	}
	w_curr(0) = imu_data.angular_velocity.x;
	w_curr(1) = imu_data.angular_velocity.y;
	w_curr(2) = imu_data.angular_velocity.z;

	Eigen::Vector3d torque_out = control_rates(w_des, w_curr);
	geometry_msgs::Vector3Stamped msg;
	msg.x = thrust_out(0);
	msg.y = thrust_out(1);
	msg.z = thrust_out(2);
	torque_pub.publish(msg);
}

Eigen::Vector3d control_attitude(const Eigen::Quaternion q_des, const Eigen::Quaternion q){
	Eigen::Quaternion q_err = q.inverse()*q_des;
	// Add feedforward if we need trajectory tracking
	if (q_err.w() >= 0){
		return 2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	else {
		return -2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	
}

Eigen::Vector3d control_rates(Eigen::Vector3d w_des, Eigen::Vector3d w){
	return 1/T_W*J*(w_des-w)+w.cross(J*w);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "attitude_controller");
	ros::NodeHandle nh;

	ros::Subscriber att_sp_sub = nh.subscribe("att_sp",1,setpointCallback);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);
	torque_pub = nh.advertise<geometry_msgs::Vector3Stamped>("torque_sp",0);

	ros::spin();
}