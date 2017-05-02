#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <omnicopter_sim/RCInput.h>
#include <omnicopter_sim/ControlMode.h>
#include <cmath>

#define ROLL_STICK 0 // 1.0 to left, -1.0 to right
#define PITCH_STICK 1 // 1.0 at bottom, -1.0 at top
#define THROTTLE_STICK 2 // 1.0 at bottom, -1.0 at top
#define YAWRATE_STICK 3 // 1.0 to left, -1.0 to right
#define ARM_CH 4 // disarmed = 0.0, armed = 1.0
#define MODE_CH1 5 //-1.0 at bottom, 0.0 at middle, 1.0 at top
#define MODE_CH2 6 //-1.0 at bottom, 0.0 at middle, 1.0 at top

/*
Note that the ROS joy node initializes all outputs to 0.
Each channel will only have the correct values after it has changed value once
This is not a problem with a real setup using a FrSky receiver
*/



ros::Publisher rc_pub;
bool rc_initialized = false;

void joyCallback(const sensor_msgs::Joy& input) {
	omnicopter_sim::RCInput msg;
	msg.header = input.header;
	msg.rollstick = -input.axes[ROLL_STICK];
	msg.pitchstick = -input.axes[PITCH_STICK];
	msg.throttlestick = -input.axes[THROTTLE_STICK];
	msg.yawstick = -input.axes[YAWRATE_STICK];
	msg.arm = (bool)round(input.axes[ARM_CH]);

	int mode1 = round(input.axes[MODE_CH1]);
	int mode2 = round(input.axes[MODE_CH2]);
	if(mode1 == 1 && mode2 == 1){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_RP_ATT_Y_RATE;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_FORCE_BODY_UP;
	}
	else if(mode1 == 1 && mode2 == 0){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_YAWRATE;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_FORCE_BODY;
	}
	else if(mode1 == 1 && mode2 == -1){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_YAWRATE;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_FORCE_ENU;
	}
	else if(mode1 != 1){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_RATES;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_FORCE_BODY_UP;
	}/*
	else if(mode1 == 0 && mode2 == 0){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_YAWRATE;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_ALTHOLD_FORCE_BODY;
	}
	else if(mode1 == 0 && mode2 == -1){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_YAWRATE;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_ALTHOLD_FORCE_ENU;
	}
	else if(mode1 == -1){
		msg.rc_mode.attitude_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_RATES;
		msg.rc_mode.position_control_mode = omnicopter_sim::ControlMode::MODE_CONTROL_FORCE_ENU;
	}*/
	

	if(rc_initialized){
		rc_pub.publish(msg);
	}
	else{
		if (msg.throttlestick < -0.9 && mode1 == 1 && mode1 == 1 && !msg.arm){
			rc_initialized = true;
		}
	}
	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "rc_mapping");
	ros::NodeHandle nh;

	ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCallback); 
	rc_pub = nh.advertise<omnicopter_sim::RCInput>("rc_input",0);

	ros::spin();
}