#include <ros/ros.h>
#include <omnicopter_sim/PosSp.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::Publisher force_pub;

void setpointCallback(const omnicopter_sim::PosSp& input){

}

int main(int argc, char **argv){
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle nh;

	ros::Subscriber pos_sp_sub = nh.subscribe("pos_sp",1,setpointCallback);
	force_pub = nh.advertise<geometry_msgs::Vector3Stamped>("force_sp",0);

	ros::spin();
}