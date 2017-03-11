/*
Responsibilities of commander:
- Mode switching based on rc switches
- Arm/disarm
	Continously send arm message to PWM driver so we disarm in case commander dies
- Failsafe
	- Estimate timeout
	- RC timeout

Notes:
- Position and velocity control modes should only be selectable if
position/velocity estimates are available (= tracking system)
- Climb rate and alt hold should only be selectable if altitude
estimates are available (=ultrasonic rangefinder)

- For pure force setpoints, do not allow negative z thrust. Create a 
two part linear throttle curve so that centered throttle gives hover thrust
(50% throttle should give m*9.81 N upwards)
- For pos/vel/althold/climbrate modes centered throttle stick is neutral
*/

#include <ros/ros.h>
#include <omnicopter_sim/AttSp.h>
#include <omnicopter_sim/PosSp.h>
#include <omnicopter_sim/RCInput.h>
#include <omnicopter_sim/State.h>
#include <omnicopter_sim/Arm.h>
#include <sensor_msgs/Imu.h>

const ros::Duration RC_TIMEOUT(0.3);
const ros::Duration IMU_TIMEOUT(0.1);
const ros::Duration POS_TIMEOUT(0.2);

bool rc_available = false;
bool imu_available = false;
bool pos_available = false;



ros::Publisher att_sp_pub, pos_sp_pub, state_pub;

ros::Time last_rc_time, last_imu_time, last_pos_time;

void rcCallback(const omnicopter_sim::RCInput& input){
	// Reset timeout clock
	last_rc_time = ros::Time::now();
	rc_available = true;
}

void imuCallback(const sensor_msgs::imu& input){
	// Reset timeout clock
	last_imu_time = ros::Time::now();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "commander");
	ros::NodeHandle nh;
	att_sp_pub = nh.advertise<omnicopter_sim::AttSp>("att_sp",0);
	pos_sp_pub = nh.advertise<omnicopter_sim::PosSp>("pos_sp",0);
	pos_sp_pub = nh.advertise<omnicopter_sim::State>("state",0);

	ros::Subscriber rc_input_sub = nh.subscribe("rc_input",1,rcCallback);
	ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);



	ros::Rate loop_rate(100);
	while(ros::ok()){
		//Check for timeouts
		if(rc_available && (ros::Time::now() - last_rc_time) < RC_TIMEOUT){
			// Handle RC timeout
			rc_available = false;
		}
		if(imu_available && (ros::Time::now() - last_imu_time) < IMU_TIMEOUT){
			// Handle IMU timeout
			imu_available = false;
		}
		if(pos_available && (ros::Time::now() - last_pos_time) < POS_TIMEOUT){
			// Handle position estimate timeout
			pos_available = false;
		}




		loop_rate.sleep();
		ros::spinOnce();
	}
}