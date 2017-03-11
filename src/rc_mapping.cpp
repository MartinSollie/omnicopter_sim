#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <omnicopter_sim/RCInput.h>
#include <geometry_msgs/Vector3Stamped.h>

#define PITCH_STICK 1 // 1.0 at bottom, -1.0 at top
#define ROLL_STICK 0 // 1.0 to left, -1.0 to right
#define THROTTLE_STICK 2 // 1.0 at bottom, -1.0 at top
#define YAWRATE_STICK 3 // 1.0 to left, -1.0 to right
#define POS_ATT_MODE_SWITCH 5 //1.0 at bottom, 0.0 at middle, -1.0 at top

#define ROLL_SCALE -1.0
#define PITCH_SCALE -1.0
#define YAWRATE_SCALE -1.0
#define THROTTLE_SCALE -1.0

ros::Publisher rc_pub;
ros::Publisher cmd_pub;

void joyCallback(const sensor_msgs::Joy& input) {
	omnicopter_sim::RCInput msg;
	msg.header = input.header;
	msg.rollstick = ROLL_SCALE*input.axes[ROLL_STICK];
	msg.pitchstick = PITCH_SCALE*input.axes[PITCH_STICK];
	msg.throttlestick = THROTTLE_SCALE*input.axes[THROTTLE_STICK];
	msg.yawstick = YAWRATE_SCALE*input.axes[YAWRATE_STICK];
	rc_pub.publish(msg);


	//Testing
	geometry_msgs::Vector3Stamped cmd;
	cmd.header = input.header;
	cmd.vector.z = 10*msg.yawrate;
	cmd.vector.x = 10*msg.rollrate;
	cmd.vector.y = 10*msg.pitchrate;
	cmd_pub.publish(cmd);


}

int main(int argc, char **argv){
	ros::init(argc, argv, "rc_mapping");
	ros::NodeHandle nh;

	ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCallback); 
	rc_pub = nh.advertise<omnicopter_sim::RCInput>("rc_input",0);
	cmd_pub = nh.advertise<geometry_msgs::Vector3Stamped>("torque_sp",0); //Testing

	ros::spin();
}