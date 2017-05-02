#include <ros/ros.h>
#include <omnicopter_sim/RCInput.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <stdio.h>

#define MAX_ROLL 45*M_PI/180
#define MAX_PITCH 45*M_PI/180

#define MAX_PITCHRATE_CMD 2.0f
#define MAX_ROLLRATE_CMD 2.0f
#define MAX_YAWRATE_CMD 1.5f

ros::Publisher torque_pub;
float T_ATT = 1.0; // Attitude control time constant

bool imu_received = false;
bool setpoint_received = false;
omnicopter_sim::RCInput setp;

bool hold_attitude, hold_yaw;
Eigen::Quaterniond q_hold;
void doControl();
Eigen::Vector3d control_attitude(const Eigen::Quaterniond q_des, const Eigen::Quaterniond q);
Eigen::Vector3d control_rates(const Eigen::Vector3d w_des, const Eigen::Vector3d w);
float yaw_h = 0;
Eigen::Quaterniond q_setp;
Eigen::Quaterniond q_err;

Eigen::Quaterniond q_des;
Eigen::Quaterniond q_curr;
Eigen::Vector3d w_des;
Eigen::Vector3d w_curr;

// PID stuff
ros::Time last_pid_time;
bool timer_set = false;
Eigen::Vector3d w_prev(0,0,0);
Eigen::Vector3d rates_int(0,0,0);
float max_int = 0.5;
Eigen::Vector3d K_p(0.9,0.9,0.9); // Roll, pitch, yaw
Eigen::Vector3d K_d(0,0,0);
Eigen::Vector3d K_i(0,0,0);

void imuCallback(const sensor_msgs::Imu& input){
	q_curr.x() = input.orientation.x;
	q_curr.y() = input.orientation.y;
	q_curr.z() = input.orientation.z;
	q_curr.w() = input.orientation.w;
	w_curr(0) = input.angular_velocity.x;
	w_curr(1) = input.angular_velocity.y;
	w_curr(2) = input.angular_velocity.z;
	if(!imu_received){
		imu_received = true;
	}
	if(setpoint_received){
		doControl();
	}

}

void setpointCallback(const omnicopter_sim::RCInput& input){
	setp = input;
	if(!setpoint_received){
		setpoint_received = true;
	}
}

Eigen::Quaterniond RPquaternionFromRC(const omnicopter_sim::RCInput& input, bool zeroRP, bool useYawrate){
	float roll = zeroRP ? 0 : MAX_ROLL*input.rollstick;
	float pitch = zeroRP ? 0 : MAX_PITCH*input.pitchstick;
	if(useYawrate){
		if(setp.throttlestick >= -0.98 && (setp.yawstick > 0.02 || setp.yawstick < -0.02)){
			yaw_h -= input.yawstick*0.03;
		}
		while(yaw_h > M_PI){
			yaw_h -= 2*M_PI;
		}
		while(yaw_h < -M_PI){
			yaw_h += 2*M_PI;
		}
	} else {
		yaw_h = M_PI*input.rollstick;
	}
	q_setp = Eigen::AngleAxisd(yaw_h,  Eigen::Vector3d::UnitZ())
	* Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
	* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q_setp;
}

Eigen::Vector3d ratesFromRC(const omnicopter_sim::RCInput& input){
	return Eigen::Vector3d(input.rollstick*MAX_ROLLRATE_CMD, input.pitchstick*MAX_PITCHRATE_CMD, -input.yawstick*MAX_YAWRATE_CMD);
}

void doControl(){
	if(setp.rc_mode.attitude_control_mode == omnicopter_sim::ControlMode::MODE_CONTROL_ATT){
		//Do attitude control
		hold_attitude = false;
		hold_yaw = false;
		q_des = RPquaternionFromRC(setp, false, false);
		w_des = control_attitude(q_des, q_curr);
	}
	else if(setp.rc_mode.attitude_control_mode == omnicopter_sim::ControlMode::MODE_CONTROL_RP_ATT_Y_RATE){
		Eigen::Vector3d w_rc = ratesFromRC(setp);
		w_des = control_attitude(RPquaternionFromRC(setp, false, true), q_curr);
	}
	else if(setp.rc_mode.attitude_control_mode == omnicopter_sim::ControlMode::MODE_CONTROL_YAWRATE){
		Eigen::Vector3d w_rc = ratesFromRC(setp);
		w_des = control_attitude(RPquaternionFromRC(setp, true, true), q_curr);

	}
	else if(setp.rc_mode.attitude_control_mode == omnicopter_sim::ControlMode::MODE_CONTROL_RATES){
		// If the rate setpoint is ~0, we want to hold the current attitude, but only sample it when the
		// actual rate gets small so we don't get a big bounceback
		Eigen::Vector3d w_rc = ratesFromRC(setp);
		bool zero_setpoint = std::abs(w_rc(0)) < 0.001 && std::abs(w_rc(1)) < 0.001 && std::abs(w_rc(2)) < 0.001;
		bool small_rate = std::abs(w_curr(0)) < 0.5 && 
							std::abs(w_curr(1)) < 0.5 && 
							std::abs(w_curr(2)) < 0.5;
		if(zero_setpoint && small_rate){
			if(!hold_attitude){
				q_hold = q_curr;
				hold_attitude = true;
			}
			w_des = control_attitude(q_hold, q_curr);

		}
		else{
			// Do rate control
			hold_attitude = false;
			w_des = w_rc;
			
		}
	}
	else{
		printf("Unknown attitude control mode!\n");
		return;
	}
	Eigen::Vector3d torque_out = control_rates(w_des, w_curr);
	if(setp.throttlestick > -0.3){
		// Update integral
		Eigen::Vector3d new_rates_int = rates_int + K_i.cwiseProduct(w_des-w_curr)*(ros::Time::now()-last_pid_time).toSec();
		for(int i = 0; i < 3; i++){
			if(new_rates_int(i) < max_int && new_rates_int(i) > -max_int){
				rates_int(i) = new_rates_int(i);
			}
		}
	}
	last_pid_time = ros::Time::now();
	geometry_msgs::Vector3Stamped msg;
	msg.header.stamp = ros::Time::now();
	msg.vector.x = torque_out(0);
	msg.vector.y = torque_out(1);
	msg.vector.z = torque_out(2);
	if(setp.throttlestick < -0.985){
		msg.vector.x = 0;
		msg.vector.y = 0;
		msg.vector.z = 0;
		rates_int = Eigen::Vector3d(0,0,0);
	}
	//printf("Torque: % 04.2f % 04.2f % 04.2f, Integral: % 04.3f % 04.3f % 04.3f\n", torque_out(0), torque_out(1), torque_out(2), rates_int(0), rates_int(1), rates_int(2));
	torque_pub.publish(msg);
}

Eigen::Vector3d control_attitude(const Eigen::Quaterniond q_des, const Eigen::Quaterniond q){
	q_err = q.inverse()*q_des;

	if (q_err.w() >= 0){
		return 2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	else {
		return -2/T_ATT*Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
	}
	
}

Eigen::Vector3d control_rates(const Eigen::Vector3d w_des, const Eigen::Vector3d w){
	// standard PID controller
	Eigen::Vector3d out;
	Eigen::Vector3d w_err = w_des-w;
	out = K_p.cwiseProduct(w_err) + K_d.cwiseProduct((w_prev-w)/(ros::Time::now()-last_pid_time).toSec()) + rates_int;
	w_prev = w;
	return out;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "attitude_controller");
	ros::NodeHandle nh;
	ros::NodeHandle n_priv("~");

	n_priv.getParam("T_ATT", T_ATT);
	n_priv.getParam("K_p_roll",K_p(0));
	n_priv.getParam("K_p_pitch",K_p(1));
	n_priv.getParam("K_p_yaw",K_p(2));
	n_priv.getParam("K_d_roll",K_d(0));
	n_priv.getParam("K_d_pitch",K_d(1));
	n_priv.getParam("K_d_yaw",K_d(2));
	n_priv.getParam("K_i_roll",K_i(0));
	n_priv.getParam("K_i_pitch",K_i(1));
	n_priv.getParam("K_i_yaw",K_i(2));

	ros::Subscriber rc_sub = nh.subscribe("rc_input",1,setpointCallback);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);
	torque_pub = nh.advertise<geometry_msgs::Vector3Stamped>("torque_sp",0);

	ros::spin();
}