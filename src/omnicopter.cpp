#include <ros/ros.h>
#include <Eigen/Eigen>
#include <omnicopter_sim/MotorCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <vector>
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

const Eigen::Matrix3d J = Eigen::Matrix3d::Identity(); //Inertia matrix
const double m = 0.5; //mass [kg]
const Eigen::Vector3d g(0,0,9.81);

visualization_msgs::MarkerArray visuals;

Eigen::MatrixXd X(3,8);
Eigen::MatrixXd P(3,8);

ros::Publisher vis_pub, pose_pub;


void commandCallback(const omnicopter_sim::MotorCommand& input) {
	motor1_cmd_usec = input.motor1_usec;
	motor2_cmd_usec = input.motor2_usec;
	motor3_cmd_usec = input.motor3_usec;
	motor4_cmd_usec = input.motor4_usec;
	motor5_cmd_usec = input.motor5_usec;
	motor6_cmd_usec = input.motor6_usec;
	motor7_cmd_usec = input.motor7_usec;
	motor8_cmd_usec = input.motor8_usec;
}

double pwmToForce(double pwm){
	if(pwm < 1000){
		return 0;
	}
	else if (pwm > 2000){
		ROS_ERROR("Simulation PWM input > 2000");
		return 0;
	}
	else{
		// TODO proper model

		// For now just assume linear pwm to speed relationship
		// and quadratic speed to force relationship
		double max_rpm = 38000;
		double max_w = max_rpm*2*PI;
		double w = max_w*(pwm-1500)/500;
		return K_F*w*w;
	}
}

double pwmToTorque(double pwm){
	if(pwm < 1000){
		return 0;
	}
	else if (pwm > 2000){
		ROS_ERROR("Simulation PWM input > 2000");
		return 0;
	}
	else{
		// TODO proper model

		// For now just assume linear pwm to speed relationship
		// and quadratic speed to torque relationship
		double max_rpm = 38000;
		double max_w = max_rpm*2*PI;
		double w = max_w*(pwm-1500)/5000;
		return K_M*w*w;
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
	return t;
}

Eigen::Vector3d getBodyForce(){
	return X*getMotorForces();
}

Eigen::Vector3d getBodyTorque(){
	Eigen::MatrixXd P_cross_X(3,8);
	for (int i = 0; i < 8; i++){
		Eigen::Vector3d P_col = P.col(i);
		Eigen::Vector3d X_col = X.col(i);
		P_cross_X.col(i) = P_col.cross(X_col);
	}
	return P_cross_X*getMotorForces() + X*getMotorTorques();
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
  	marker.pose.orientation.x = q.x();
  	marker.pose.orientation.y = q.y();
  	marker.pose.orientation.z = q.z();
  	marker.pose.orientation.w = q.w();
  	marker.scale.x = 0.3;
  	marker.scale.y = 0.3;
  	marker.scale.z = 0.3;
  	marker.color.a = 1.0; // Don't forget to set the alpha!
  	marker.color.r = 0.0;
  	marker.color.g = 1.0;
  	marker.color.b = 0.0;
  	visuals.markers.push_back(marker);
}

void publishVisualization(){
	// Update motor forces and torques
	for (int i = 0; i < visuals.markers.size(); i++){
		visuals.markers[i].header.stamp = ros::Time::now();
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
	msg.pose.orientation.x = q.x();
	msg.pose.orientation.y = q.y();
	msg.pose.orientation.z = q.z();
	msg.pose.orientation.w = q.w();
	pose_pub.publish(msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "omnicopter");
	ros::NodeHandle nh;

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

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
	Eigen::Vector3d acc(0,0,0); //linear acceleration
	Eigen::Vector3d alpha(0,0,0); //angular acceleration

	while(ros::ok()) {

		ros::spinOnce();

		Eigen::Matrix3d R = q.normalized().toRotationMatrix();
		f = getBodyForce();
		t = getBodyTorque();

		// Kinetics
		acc = (1/m)*R*f ;//- g;
		alpha = J.inverse()*(t-w.cross(J*w));

		// Kinematics
		// Linear
		v += acc*dt;
		p += v*dt;

		// Angular
		w += alpha*dt; 
		Eigen::Quaterniond w_q(0,w(0)*0.5*dt,w(1)*0.5*dt,w(2)*0.5*dt);
		Eigen::Quaterniond q_dot = q*w_q;
		q.x() += q_dot.x();
		q.y() += q_dot.y();
		q.z() += q_dot.z();
		q.w() += q_dot.w();
		q.normalize();


		publishPose();
		publishVisualization(); //Maybe do this at fixed 60Hz?
		loop_rate.sleep();
	}

}