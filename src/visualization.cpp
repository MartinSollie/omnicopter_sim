#include <ros/ros.h>
#include <omnicopter_sim/MotorCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf/tf.h>
#include <vector>
#include <Eigen/Geometry>

#define K_F 6.8834e-11
#define K_M 6.8834e-12 //wild guess
#define PI 3.141592653589
#define MOTOR_DISTANCE_CG 0.12
#define BOX_DIM 0.4

double pwmToForce(double pwm);
double pwmToTorque(double pwm);


ros::Publisher vis_pub;
visualization_msgs::MarkerArray visuals;



double pwmToForce(double pwm){
	if(pwm < 1000){
		return 0;
	}
	else if (pwm > 2000){
		ROS_ERROR("Simulation PWM input > 2000");
		return 0;
	}
	else{
		double max_rpm = 38000;
		double max_w = max_rpm*2*PI;
		double w = max_w*(pwm-1500)/500;
		if(w < 0){
			return -K_F*w*w;
		}
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
		double max_rpm = 38000;
		double max_w = max_rpm*2*PI;
		double w = max_w*(pwm-1500)/500;
		if(w < 0){
			return -K_M*w*w;
		}
		return K_M*w*w;
	}
}

const double MAX_FORCE(pwmToForce(2000));

void commandCallback(const omnicopter_sim::MotorCommand& input) {
	// Update motor forces
	visuals.markers[5].scale.x = 0.2*pwmToForce(input.motor1_usec)/MAX_FORCE;
	visuals.markers[6].scale.x = 0.2*pwmToForce(input.motor2_usec)/MAX_FORCE;
	visuals.markers[7].scale.x = 0.2*pwmToForce(input.motor3_usec)/MAX_FORCE;
	visuals.markers[8].scale.x = 0.2*pwmToForce(input.motor4_usec)/MAX_FORCE;
	visuals.markers[9].scale.x = 0.2*pwmToForce(input.motor5_usec)/MAX_FORCE;
	visuals.markers[10].scale.x = 0.2*pwmToForce(input.motor6_usec)/MAX_FORCE;
	visuals.markers[11].scale.x = 0.2*pwmToForce(input.motor7_usec)/MAX_FORCE;
	visuals.markers[12].scale.x = 0.2*pwmToForce(input.motor8_usec)/MAX_FORCE;

	// Update motor torques

	for (int i = 5; i <= 12 ; i++){
		if(visuals.markers[i].scale.x > 0){
			visuals.markers[i].color.r = 0.0;
	  		visuals.markers[i].color.g = 1.0;

		} else {
			visuals.markers[i].color.r = 1.0;
	  		visuals.markers[i].color.g = 0.0;
		}
	}

	for(int i = 0; i < visuals.markers.size(); i++){
		visuals.markers[i].header.stamp = ros::Time::now();
	}

	vis_pub.publish(visuals);
}


void initializeVisualization(){
	//Draw a box
	visualization_msgs::Marker box;
	box.header.frame_id = "body";
	box.header.stamp = ros::Time::now();
	box.ns = "omnicopter";
	box.id = 0;
	box.type = visualization_msgs::Marker::CUBE;
   	box.action = visualization_msgs::Marker::ADD;
	box.pose.position.x = 0;
   	box.pose.position.y = 0;
  	box.pose.position.z = 0;
  	box.pose.orientation.x = 0;
  	box.pose.orientation.y = 0;
  	box.pose.orientation.z = 0;
  	box.pose.orientation.w = 1;
  	box.scale.x = BOX_DIM;
  	box.scale.y = BOX_DIM;
  	box.scale.z = BOX_DIM;
  	box.color.a = 0.2;
  	box.color.r = 0.0;
  	box.color.g = 1.0;
  	box.color.b = 0.0;
  	visuals.markers.push_back(box);

  	//Draw 4 carbon tubes
  	tf::Quaternion q_1;
  	tf::Quaternion q_2;
  	tf::Quaternion q_3;
  	tf::Quaternion q_4;
  	q_1.setRotation(tf::Vector3(1, 1, 0),0.95531662);
  	q_2.setRotation(tf::Vector3(1, -1, 0),0.95531662);
  	q_3.setRotation(tf::Vector3(-1, 1, 0),0.95531662);
  	q_4.setRotation(tf::Vector3(-1, -1, 0),0.95531662);
  	
  	geometry_msgs::Quaternion q1_msg;
  	geometry_msgs::Quaternion q2_msg;
  	geometry_msgs::Quaternion q3_msg;
  	geometry_msgs::Quaternion q4_msg;
  	tf::quaternionTFToMsg(q_1, q1_msg);
  	tf::quaternionTFToMsg(q_2, q2_msg);
  	tf::quaternionTFToMsg(q_3, q3_msg);
  	tf::quaternionTFToMsg(q_4, q4_msg);

  	visualization_msgs::Marker tube1;
	tube1.header.frame_id = "body";
	tube1.header.stamp = ros::Time::now();
	tube1.ns = "omnicopter";
	tube1.id = 1;
	tube1.type = visualization_msgs::Marker::CYLINDER;
   	tube1.action = visualization_msgs::Marker::ADD;
   	tube1.pose.position.x = 0;
	tube1.pose.position.y = 0;
	tube1.pose.position.z = 0;
	tube1.pose.orientation = q1_msg;
	tube1.scale.x = 0.008;
  	tube1.scale.y = 0.008;
  	tube1.scale.z = BOX_DIM*sqrt(3);
  	tube1.color.a = 0.7;
  	tube1.color.r = 1.0;
  	tube1.color.g = 1.0;
  	tube1.color.b = 1.0;
  	visuals.markers.push_back(tube1);

  	visualization_msgs::Marker tube2;
	tube2.header.frame_id = "body";
	tube2.header.stamp = ros::Time::now();
	tube2.ns = "omnicopter";
	tube2.id = 2;
	tube2.type = visualization_msgs::Marker::CYLINDER;
   	tube2.action = visualization_msgs::Marker::ADD;
   	tube2.pose.position.x = 0;
	tube2.pose.position.y = 0;
	tube2.pose.position.z = 0;
	tube2.pose.orientation = q2_msg;
	tube2.scale.x = 0.008;
  	tube2.scale.y = 0.008;
  	tube2.scale.z = BOX_DIM*sqrt(3);
  	tube2.color.a = 0.7;
  	tube2.color.r = 1.0;
  	tube2.color.g = 1.0;
  	tube2.color.b = 1.0;
  	visuals.markers.push_back(tube2);

  	visualization_msgs::Marker tube3;
	tube3.header.frame_id = "body";
	tube3.header.stamp = ros::Time::now();
	tube3.ns = "omnicopter";
	tube3.id = 3;
	tube3.type = visualization_msgs::Marker::CYLINDER;
   	tube3.action = visualization_msgs::Marker::ADD;
   	tube3.pose.position.x = 0;
	tube3.pose.position.y = 0;
	tube3.pose.position.z = 0;
	tube3.pose.orientation = q3_msg;
	tube3.scale.x = 0.008;
  	tube3.scale.y = 0.008;
  	tube3.scale.z = BOX_DIM*sqrt(3);
  	tube3.color.a = 0.5;
  	tube3.color.r = 1.0;
  	tube3.color.g = 1.0;
  	tube3.color.b = 1.0;
  	visuals.markers.push_back(tube3);

  	visualization_msgs::Marker tube4;
	tube4.header.frame_id = "body";
	tube4.header.stamp = ros::Time::now();
	tube4.ns = "omnicopter";
	tube4.id = 4;
	tube4.type = visualization_msgs::Marker::CYLINDER;
   	tube4.action = visualization_msgs::Marker::ADD;
   	tube4.pose.position.x = 0;
	tube4.pose.position.y = 0;
	tube4.pose.position.z = 0;
	tube4.pose.orientation = q4_msg;
	tube4.scale.x = 0.008;
  	tube4.scale.y = 0.008;
  	tube4.scale.z = BOX_DIM*sqrt(3);
  	tube4.color.a = 0.5;
  	tube4.color.r = 1.0;
  	tube4.color.g = 1.0;
  	tube4.color.b = 1.0;
  	visuals.markers.push_back(tube4);

  	// Draw 8 arrows for motor thrust (green for positive thrust, red for negative)
  	double a = 0.5 + 1.0/sqrt(12);
	double b = 0.5 - 1.0/sqrt(12);
	double c = 1/sqrt(3);
	Eigen::MatrixXd X(3,8);
	X << -a, b,-b, a, a,-b, b,-a,
		  b, a,-a,-b,-b,-a, a, b,
		  c,-c,-c, c, c,-c,-c, c;

	Eigen:: Vector3d v_base(1,0,0);

	Eigen::Vector3d v1 = X.col(0);
	Eigen::Vector3d v2 = X.col(1);
	Eigen::Vector3d v3 = X.col(2);
	Eigen::Vector3d v4 = X.col(3);
	Eigen::Vector3d v5 = X.col(4);
	Eigen::Vector3d v6 = X.col(5);
	Eigen::Vector3d v7 = X.col(6);
	Eigen::Vector3d v8 = X.col(7);
	Eigen::Vector3d c1 = v_base.cross(v1);
	Eigen::Vector3d c2 = v_base.cross(v2);
	Eigen::Vector3d c3 = v_base.cross(v3);
	Eigen::Vector3d c4 = v_base.cross(v4);
	Eigen::Vector3d c5 = v_base.cross(v5);
	Eigen::Vector3d c6 = v_base.cross(v6);
	Eigen::Vector3d c7 = v_base.cross(v7);
	Eigen::Vector3d c8 = v_base.cross(v8);

	tf::Quaternion q_m1(c1(0), c1(1), c1(2), sqrt(v_base.norm()*v_base.norm()*v1.norm()*v1.norm())+v_base.dot(v1));
	tf::Quaternion q_m2(c2(0), c2(1), c2(2), sqrt(v_base.norm()*v_base.norm()*v2.norm()*v2.norm())+v_base.dot(v2));
	tf::Quaternion q_m3(c3(0), c3(1), c3(2), sqrt(v_base.norm()*v_base.norm()*v3.norm()*v3.norm())+v_base.dot(v3));
	tf::Quaternion q_m4(c4(0), c4(1), c4(2), sqrt(v_base.norm()*v_base.norm()*v4.norm()*v4.norm())+v_base.dot(v4));
	tf::Quaternion q_m5(c5(0), c5(1), c5(2), sqrt(v_base.norm()*v_base.norm()*v5.norm()*v5.norm())+v_base.dot(v5));
	tf::Quaternion q_m6(c6(0), c6(1), c6(2), sqrt(v_base.norm()*v_base.norm()*v6.norm()*v6.norm())+v_base.dot(v6));
	tf::Quaternion q_m7(c7(0), c7(1), c7(2), sqrt(v_base.norm()*v_base.norm()*v7.norm()*v7.norm())+v_base.dot(v7));
	tf::Quaternion q_m8(c8(0), c8(1), c8(2), sqrt(v_base.norm()*v_base.norm()*v8.norm()*v8.norm())+v_base.dot(v8));

  	geometry_msgs::Quaternion qm1_msg;
  	geometry_msgs::Quaternion qm2_msg;
  	geometry_msgs::Quaternion qm3_msg;
  	geometry_msgs::Quaternion qm4_msg;
  	geometry_msgs::Quaternion qm5_msg;
  	geometry_msgs::Quaternion qm6_msg;
  	geometry_msgs::Quaternion qm7_msg;
  	geometry_msgs::Quaternion qm8_msg;
  	tf::quaternionTFToMsg(q_m1.normalized(), qm1_msg);
  	tf::quaternionTFToMsg(q_m2.normalized(), qm2_msg);
  	tf::quaternionTFToMsg(q_m3.normalized(), qm3_msg);
  	tf::quaternionTFToMsg(q_m4.normalized(), qm4_msg);
  	tf::quaternionTFToMsg(q_m5.normalized(), qm5_msg);
  	tf::quaternionTFToMsg(q_m6.normalized(), qm6_msg);
  	tf::quaternionTFToMsg(q_m7.normalized(), qm7_msg);
  	tf::quaternionTFToMsg(q_m8.normalized(), qm8_msg);


  	visualization_msgs::Marker thrust1;
	thrust1.header.frame_id = "body";
	thrust1.header.stamp = ros::Time::now();
	thrust1.ns = "omnicopter";
	thrust1.id = 5;
	thrust1.type = visualization_msgs::Marker::ARROW;
   	thrust1.action = visualization_msgs::Marker::ADD;
   	thrust1.pose.position.x = MOTOR_DISTANCE_CG;
	thrust1.pose.position.y = MOTOR_DISTANCE_CG;
	thrust1.pose.position.z = MOTOR_DISTANCE_CG;
	thrust1.pose.orientation = qm1_msg;
	thrust1.scale.x = 0.2;
  	thrust1.scale.y = 0.01;
  	thrust1.scale.z = 0.01;
  	thrust1.color.a = 1.0;
  	thrust1.color.r = 1.0;
  	thrust1.color.g = 0.0;
  	thrust1.color.b = 0.0;
  	visuals.markers.push_back(thrust1);

  	visualization_msgs::Marker thrust2;
	thrust2.header.frame_id = "body";
	thrust2.header.stamp = ros::Time::now();
	thrust2.ns = "omnicopter";
	thrust2.id = 6;
	thrust2.type = visualization_msgs::Marker::ARROW;
   	thrust2.action = visualization_msgs::Marker::ADD;
   	thrust2.pose.position.x = -MOTOR_DISTANCE_CG;
	thrust2.pose.position.y = MOTOR_DISTANCE_CG;
	thrust2.pose.position.z = MOTOR_DISTANCE_CG;
	thrust2.pose.orientation = qm2_msg;
	thrust2.scale.x = 0.2;
  	thrust2.scale.y = 0.01;
  	thrust2.scale.z = 0.01;
  	thrust2.color.a = 1.0;
  	thrust2.color.r = 1.0;
  	thrust2.color.g = 0.0;
  	thrust2.color.b = 0.0;
  	visuals.markers.push_back(thrust2);

  	visualization_msgs::Marker thrust3;
	thrust3.header.frame_id = "body";
	thrust3.header.stamp = ros::Time::now();
	thrust3.ns = "omnicopter";
	thrust3.id = 7;
	thrust3.type = visualization_msgs::Marker::ARROW;
   	thrust3.action = visualization_msgs::Marker::ADD;
   	thrust3.pose.position.x = MOTOR_DISTANCE_CG;
	thrust3.pose.position.y = -MOTOR_DISTANCE_CG;
	thrust3.pose.position.z = MOTOR_DISTANCE_CG;
	thrust3.pose.orientation = qm3_msg;
	thrust3.scale.x = 0.2;
  	thrust3.scale.y = 0.01;
  	thrust3.scale.z = 0.01;
  	thrust3.color.a = 1.0;
  	thrust3.color.r = 1.0;
  	thrust3.color.g = 0.0;
  	thrust3.color.b = 0.0;
  	visuals.markers.push_back(thrust3);

  	visualization_msgs::Marker thrust4;
	thrust4.header.frame_id = "body";
	thrust4.header.stamp = ros::Time::now();
	thrust4.ns = "omnicopter";
	thrust4.id = 8;
	thrust4.type = visualization_msgs::Marker::ARROW;
   	thrust4.action = visualization_msgs::Marker::ADD;
   	thrust4.pose.position.x = -MOTOR_DISTANCE_CG;
	thrust4.pose.position.y = -MOTOR_DISTANCE_CG;
	thrust4.pose.position.z = MOTOR_DISTANCE_CG;
	thrust4.pose.orientation = qm4_msg;
	thrust4.scale.x = 0.2;
  	thrust4.scale.y = 0.01;
  	thrust4.scale.z = 0.01;
  	thrust4.color.a = 1.0;
  	thrust4.color.r = 1.0;
  	thrust4.color.g = 0.0;
  	thrust4.color.b = 0.0;
  	visuals.markers.push_back(thrust4);

  	visualization_msgs::Marker thrust5;
	thrust5.header.frame_id = "body";
	thrust5.header.stamp = ros::Time::now();
	thrust5.ns = "omnicopter";
	thrust5.id = 9;
	thrust5.type = visualization_msgs::Marker::ARROW;
   	thrust5.action = visualization_msgs::Marker::ADD;
   	thrust5.pose.position.x = MOTOR_DISTANCE_CG;
	thrust5.pose.position.y = MOTOR_DISTANCE_CG;
	thrust5.pose.position.z = -MOTOR_DISTANCE_CG;
	thrust5.pose.orientation = qm5_msg;
	thrust5.scale.x = 0.2;
  	thrust5.scale.y = 0.01;
  	thrust5.scale.z = 0.01;
  	thrust5.color.a = 1.0;
  	thrust5.color.r = 1.0;
  	thrust5.color.g = 0.0;
  	thrust5.color.b = 0.0;
  	visuals.markers.push_back(thrust5);

  	visualization_msgs::Marker thrust6;
	thrust6.header.frame_id = "body";
	thrust6.header.stamp = ros::Time::now();
	thrust6.ns = "omnicopter";
	thrust6.id = 10;
	thrust6.type = visualization_msgs::Marker::ARROW;
   	thrust6.action = visualization_msgs::Marker::ADD;
   	thrust6.pose.position.x = -MOTOR_DISTANCE_CG;
	thrust6.pose.position.y = MOTOR_DISTANCE_CG;
	thrust6.pose.position.z = -MOTOR_DISTANCE_CG;
	thrust6.pose.orientation = qm6_msg;
	thrust6.scale.x = 0.2;
  	thrust6.scale.y = 0.01;
  	thrust6.scale.z = 0.01;
  	thrust6.color.a = 1.0;
  	thrust6.color.r = 1.0;
  	thrust6.color.g = 0.0;
  	thrust6.color.b = 0.0;
  	visuals.markers.push_back(thrust6);

  	visualization_msgs::Marker thrust7;
	thrust7.header.frame_id = "body";
	thrust7.header.stamp = ros::Time::now();
	thrust7.ns = "omnicopter";
	thrust7.id = 11;
	thrust7.type = visualization_msgs::Marker::ARROW;
   	thrust7.action = visualization_msgs::Marker::ADD;
   	thrust7.pose.position.x = MOTOR_DISTANCE_CG;
	thrust7.pose.position.y = -MOTOR_DISTANCE_CG;
	thrust7.pose.position.z = -MOTOR_DISTANCE_CG;
	thrust7.pose.orientation = qm7_msg;
	thrust7.scale.x = 0.2;
  	thrust7.scale.y = 0.01;
  	thrust7.scale.z = 0.01;
  	thrust7.color.a = 1.0;
  	thrust7.color.r = 1.0;
  	thrust7.color.g = 0.0;
  	thrust7.color.b = 0.0;
  	visuals.markers.push_back(thrust7);

  	visualization_msgs::Marker thrust8;
	thrust8.header.frame_id = "body";
	thrust8.header.stamp = ros::Time::now();
	thrust8.ns = "omnicopter";
	thrust8.id = 12;
	thrust8.type = visualization_msgs::Marker::ARROW;
   	thrust8.action = visualization_msgs::Marker::ADD;
   	thrust8.pose.position.x = -MOTOR_DISTANCE_CG;
	thrust8.pose.position.y = -MOTOR_DISTANCE_CG;
	thrust8.pose.position.z = -MOTOR_DISTANCE_CG;
	thrust8.pose.orientation = qm8_msg;
	thrust8.scale.x = 0.2;
  	thrust8.scale.y = 0.01;
  	thrust8.scale.z = 0.01;
  	thrust8.color.a = 1.0;
  	thrust8.color.r = 1.0;
  	thrust8.color.g = 0.0;
  	thrust8.color.b = 0.0;
  	visuals.markers.push_back(thrust8);

  	// Draw 8 spheres for motors
  	visualization_msgs::Marker motor1;
	motor1.header.frame_id = "body";
	motor1.header.stamp = ros::Time::now();
	motor1.ns = "omnicopter";
	motor1.id = 13;
	motor1.type = visualization_msgs::Marker::SPHERE;
   	motor1.action = visualization_msgs::Marker::ADD;
   	motor1.pose.position.x = MOTOR_DISTANCE_CG;
	motor1.pose.position.y = MOTOR_DISTANCE_CG;
	motor1.pose.position.z = MOTOR_DISTANCE_CG;
	motor1.pose.orientation.x = 0;
	motor1.pose.orientation.y = 0;
	motor1.pose.orientation.z = 0;
	motor1.pose.orientation.w = 1;
	motor1.scale.x = 0.03;
  	motor1.scale.y = 0.03;
  	motor1.scale.z = 0.03;
  	motor1.color.a = 1.0;
  	motor1.color.r = 0.0;
  	motor1.color.g = 0.0;
  	motor1.color.b = 1.0;
  	visuals.markers.push_back(motor1);

  	visualization_msgs::Marker motor2;
	motor2.header.frame_id = "body";
	motor2.header.stamp = ros::Time::now();
	motor2.ns = "omnicopter";
	motor2.id = 14;
	motor2.type = visualization_msgs::Marker::SPHERE;
   	motor2.action = visualization_msgs::Marker::ADD;
   	motor2.pose.position.x = -MOTOR_DISTANCE_CG;
	motor2.pose.position.y = MOTOR_DISTANCE_CG;
	motor2.pose.position.z = MOTOR_DISTANCE_CG;
	motor2.pose.orientation.x = 0;
	motor2.pose.orientation.y = 0;
	motor2.pose.orientation.z = 0;
	motor2.pose.orientation.w = 1;
	motor2.scale.x = 0.03;
  	motor2.scale.y = 0.03;
  	motor2.scale.z = 0.03;
  	motor2.color.a = 1.0;
  	motor2.color.r = 0.0;
  	motor2.color.g = 0.0;
  	motor2.color.b = 1.0;
  	visuals.markers.push_back(motor2);

  	visualization_msgs::Marker motor3;
	motor3.header.frame_id = "body";
	motor3.header.stamp = ros::Time::now();
	motor3.ns = "omnicopter";
	motor3.id = 15;
	motor3.type = visualization_msgs::Marker::SPHERE;
   	motor3.action = visualization_msgs::Marker::ADD;
   	motor3.pose.position.x = MOTOR_DISTANCE_CG;
	motor3.pose.position.y = -MOTOR_DISTANCE_CG;
	motor3.pose.position.z = MOTOR_DISTANCE_CG;
	motor3.pose.orientation.x = 0;
	motor3.pose.orientation.y = 0;
	motor3.pose.orientation.z = 0;
	motor3.pose.orientation.w = 1;
	motor3.scale.x = 0.03;
  	motor3.scale.y = 0.03;
  	motor3.scale.z = 0.03;
  	motor3.color.a = 1.0;
  	motor3.color.r = 0.0;
  	motor3.color.g = 0.0;
  	motor3.color.b = 1.0;
  	visuals.markers.push_back(motor3);

  	visualization_msgs::Marker motor4;
	motor4.header.frame_id = "body";
	motor4.header.stamp = ros::Time::now();
	motor4.ns = "omnicopter";
	motor4.id = 16;
	motor4.type = visualization_msgs::Marker::SPHERE;
   	motor4.action = visualization_msgs::Marker::ADD;
   	motor4.pose.position.x = -MOTOR_DISTANCE_CG;
	motor4.pose.position.y = -MOTOR_DISTANCE_CG;
	motor4.pose.position.z = MOTOR_DISTANCE_CG;
	motor4.pose.orientation.x = 0;
	motor4.pose.orientation.y = 0;
	motor4.pose.orientation.z = 0;
	motor4.pose.orientation.w = 1;
	motor4.scale.x = 0.03;
  	motor4.scale.y = 0.03;
  	motor4.scale.z = 0.03;
  	motor4.color.a = 1.0;
  	motor4.color.r = 0.0;
  	motor4.color.g = 0.0;
  	motor4.color.b = 1.0;
  	visuals.markers.push_back(motor4);

  	visualization_msgs::Marker motor5;
	motor5.header.frame_id = "body";
	motor5.header.stamp = ros::Time::now();
	motor5.ns = "omnicopter";
	motor5.id = 17;
	motor5.type = visualization_msgs::Marker::SPHERE;
   	motor5.action = visualization_msgs::Marker::ADD;
   	motor5.pose.position.x = MOTOR_DISTANCE_CG;
	motor5.pose.position.y = MOTOR_DISTANCE_CG;
	motor5.pose.position.z = -MOTOR_DISTANCE_CG;
	motor5.pose.orientation.x = 0;
	motor5.pose.orientation.y = 0;
	motor5.pose.orientation.z = 0;
	motor5.pose.orientation.w = 1;
	motor5.scale.x = 0.03;
  	motor5.scale.y = 0.03;
  	motor5.scale.z = 0.03;
  	motor5.color.a = 1.0;
  	motor5.color.r = 0.0;
  	motor5.color.g = 0.0;
  	motor5.color.b = 1.0;
  	visuals.markers.push_back(motor5);

  	visualization_msgs::Marker motor6;
	motor6.header.frame_id = "body";
	motor6.header.stamp = ros::Time::now();
	motor6.ns = "omnicopter";
	motor6.id = 18;
	motor6.type = visualization_msgs::Marker::SPHERE;
   	motor6.action = visualization_msgs::Marker::ADD;
   	motor6.pose.position.x = -MOTOR_DISTANCE_CG;
	motor6.pose.position.y = MOTOR_DISTANCE_CG;
	motor6.pose.position.z = -MOTOR_DISTANCE_CG;
	motor6.pose.orientation.x = 0;
	motor6.pose.orientation.y = 0;
	motor6.pose.orientation.z = 0;
	motor6.pose.orientation.w = 1;
	motor6.scale.x = 0.03;
  	motor6.scale.y = 0.03;
  	motor6.scale.z = 0.03;
  	motor6.color.a = 1.0;
  	motor6.color.r = 0.0;
  	motor6.color.g = 0.0;
  	motor6.color.b = 1.0;
  	visuals.markers.push_back(motor6);

  	visualization_msgs::Marker motor7;
	motor7.header.frame_id = "body";
	motor7.header.stamp = ros::Time::now();
	motor7.ns = "omnicopter";
	motor7.id = 19;
	motor7.type = visualization_msgs::Marker::SPHERE;
   	motor7.action = visualization_msgs::Marker::ADD;
   	motor7.pose.position.x = MOTOR_DISTANCE_CG;
	motor7.pose.position.y = -MOTOR_DISTANCE_CG;
	motor7.pose.position.z = -MOTOR_DISTANCE_CG;
	motor7.pose.orientation.x = 0;
	motor7.pose.orientation.y = 0;
	motor7.pose.orientation.z = 0;
	motor7.pose.orientation.w = 1;
	motor7.scale.x = 0.03;
  	motor7.scale.y = 0.03;
  	motor7.scale.z = 0.03;
  	motor7.color.a = 1.0;
  	motor7.color.r = 0.0;
  	motor7.color.g = 0.0;
  	motor7.color.b = 1.0;
  	visuals.markers.push_back(motor7);

  	visualization_msgs::Marker motor8;
	motor8.header.frame_id = "body";
	motor8.header.stamp = ros::Time::now();
	motor8.ns = "omnicopter";
	motor8.id = 20;
	motor8.type = visualization_msgs::Marker::SPHERE;
   	motor8.action = visualization_msgs::Marker::ADD;
   	motor8.pose.position.x = -MOTOR_DISTANCE_CG;
	motor8.pose.position.y = -MOTOR_DISTANCE_CG;
	motor8.pose.position.z = -MOTOR_DISTANCE_CG;
	motor8.pose.orientation.x = 0;
	motor8.pose.orientation.y = 0;
	motor8.pose.orientation.z = 0;
	motor8.pose.orientation.w = 1;
	motor8.scale.x = 0.03;
  	motor8.scale.y = 0.03;
  	motor8.scale.z = 0.03;
  	motor8.color.a = 1.0;
  	motor8.color.r = 0.0;
  	motor8.color.g = 0.0;
  	motor8.color.b = 1.0;
  	visuals.markers.push_back(motor8);

  	// Draw 8 arrows for torque

  	vis_pub.publish(visuals);



}


int main(int argc, char **argv){
	ros::init(argc, argv, "visualization");
	ros::NodeHandle nh;
	vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );
	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback); 

	initializeVisualization();

	ros::Rate loop_rate(60);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}