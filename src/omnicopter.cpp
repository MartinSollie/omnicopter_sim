#include <ros/ros.h>
#include <Eigen/Eigen>
#include <omnicopter_sim/MotorCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <vector.h>
#include <cmath>

#define LOOP_RATE 1000
#define K_F 6.8834e-11
#define K_M 6.8834e-12 //wild guess
#define PI 3.141592653589

double motor1_cmd_usec = 0;
double motor2_cmd_usec = 0;
double motor3_cmd_usec = 0;
double motor4_cmd_usec = 0;
double motor5_cmd_usec = 0;
double motor6_cmd_usec = 0;
double motor7_cmd_usec = 0;
double motor8_cmd_usec = 0;

Eigen::Vector3d p(0,0,0); //Position
Eigen::Quaterniond q(1,0,0,0); //Attitude quaternion (w,x,y,z)
Eigen::Vector3d w(0,0,0); //Angular rates
Eigen::Vector3d v(0,0,0); //Velocity

const Eigen::Matrix3d J(1,1,1); //Inertia matrix
const double m = 0.5; //mass [kg]
const Eigen::Vector3d g(0,0,9.81);

visualization_msgs::MarkerArray visuals;

Eigen::MatrixXd X(3,8);
Eigen::MatrixXd P(3,8);


void commandCallback(const omnicopter_ros::MotorCommand& input) {
	motor1_force_cmd = input.motor1_usec;
	motor2_force_cmd = input.motor2_usec;
	motor3_force_cmd = input.motor3_usec;
	motor4_force_cmd = input.motor4_usec;
	motor5_force_cmd = input.motor5_usec;
	motor6_force_cmd = input.motor6_usec;
	motor7_force_cmd = input.motor7_usec;
	motor8_force_cmd = input.motor8_usec;
}

double pwmToForce(double pwm){
	if(pwm < 1000){
		return 0;
	}
	else if (pwm > 2000){
		ROS_ERROR("Simulation PWM input > 2000")
		return 0;
	}
	else{
		// TODO proper model

		// For now just assume linear pwm to speed relationship
		// and quadratic speed to force relationship
		double max_rpm = 38000;
		double max_w = max_rpm*2*pi;
		double w = max_w*(pwm-1500)/500;
		return K_F*w^2;
	}
}

double pwmToTorque(double pwm){
	if(pwm < 1000){
		return 0;
	}
	else if (pwm > 2000){
		ROS_ERROR("Simulation PWM input > 2000")
		return 0;
	}
	else{
		// TODO proper model

		// For now just assume linear pwm to speed relationship
		// and quadratic speed to torque relationship
		double max_rpm = 38000;
		double max_w = max_rpm*2*pi;
		double w = max_w*(pwm-1500)/5000;
		return K_M*w^2;
	}
}

Eigen::VectorXd getMotorForces(){
	Eigen::VectorXd f(8);
	f(0) = pwmToForce(motor1_cmd_usec);
	f(1) = pwmToForce(motor2_cmd_usec);
	f(2) = pwmToForce(motor3_cmd_usec);
	f(3) = pwmToForce(motor4_cmd_usec);
	f(4) = pwmToForce(motor5_cmd_usec);
	f(5) = pwmToForce(motor6_cmd_usec);
	f(6) = pwmToForce(motor7_cmd_usec);
	f(7) = pwmToForce(motor8_cmd_usec);
	return f;
}

Eigen::VectorXd getMotorTorques(){
	Eigen::VectorXd t(8);
	t(0) = pwmToTorque(motor1_cmd_usec);
	t(1) = pwmToTorque(motor2_cmd_usec);
	t(2) = pwmToTorque(motor3_cmd_usec);
	t(3) = pwmToTorque(motor4_cmd_usec);
	t(4) = pwmToTorque(motor5_cmd_usec);
	t(5) = pwmToTorque(motor6_cmd_usec);
	t(6) = pwmToTorque(motor7_cmd_usec);
	t(7) = pwmToTorque(motor8_cmd_usec);
	return f;
}

Eigen::Vector3d getBodyForce(){
	return X*getMotorForces();
}

Eigen::Vector3d getBodyTorque(){
	return (P.cross(X))*getMotorForces() + X*getMotorTorques();
}

void initializeVisualization(){
	//Draw a box
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "omnicopter";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
   	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = p(0);
   	marker.pose.position.y = p(1);
  	marker.pose.position.z = p(2);
  	marker.pose.orientation.x = q(0);
  	marker.pose.orientation.y = q(1);
  	marker.pose.orientation.z = q(2);
  	marker.pose.orientation.w = q(3);
  	marker.scale.x = 0.3;
  	marker.scale.y = 0.3;
  	marker.scale.z = 0.3;
  	marker.color.a = 1.0; // Don't forget to set the alpha!
  	marker.color.r = 0.0;
  	marker.color.g = 1.0;
  	marker.color.b = 0.0;
  	visuals.push_back(marker);
}

void publishVisualization(){
	// Update motor forces and torques
	for (int i = 0; i < visuals.size(); i++){
		visuals[i].header.stamp = ros::Time::now();
	}

	vis_pub.publish(visuals);

}

void publishPose(){
	geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "world";
	msg.pose.position.x = p(0);
	msg.pose.position.y = p(1);
	msg.pose.position.z = p(2);
	msg.pose.orientation.x = q(0);
	msg.pose.orientation.y = q(1);
	msg.pose.orientation.z = q(2);
	msg.pose.orientation.w = q(3);
	pose_pub.publish(msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "omnicopter");
	ros::NodeHandle nh();

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
	ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback); 


	double a = 0.5 + 1.0/sqrt(12);
	double b = 0.5- 1.0/sqrt(12);
	double c = sqrt(3);
	X << -a, b,-b, a, a,-b, b,-a,
		  b, a,-b,-a,-a,-b, a, b,
		  c,-c,-c, c, c,-c,-c, c;

	P << 1,-1, 1,-1, 1,-1, 1,-1,
		 1, 1,-1,-1, 1, 1,-1,-1,
		 1, 1, 1, 1,-1,-1,-1,-1;

	P *= 1.0/sqrt(3);


	ros::Rate loop_rate(LOOP_RATE);
	double dt = 1/LOOP_RATE;
	Eigen::Vector3d f(0,0,0); //total force in BODY frame
	Eigen::Vector3d t(0,0,0); //total torque in BODY frame
	Eigen::Vector3d a(0,0,0); //linear acceleration
	Eigen::Vector3d alpha(0,0,0); //angular acceleration

	while(ros::ok()) {

		ros::spinOnce();

		Matrix3d R = q.normalized().toRotationMatrix();
		f = getBodyForce();
		t = getBodyTorque();

		// Kinetics
		a = (1/m)*R*f - g;
		alpha = J.inverse()*(t-w.cross(J*w));

		// Kinematics
		// Linear
		v += a*dt;
		p += v*dt;

		// Angular
		w += alpha*dt; 
		Eigen::Quaterniond w_q(0,w(0),w(1),w(2));
		q += 0.5*q*w_q*dt;
		q.normalize();


		publishPose();
		publishVisualization(); //Maybe do this at fixed 60Hz?
		loop_rate.sleep();
	}

}