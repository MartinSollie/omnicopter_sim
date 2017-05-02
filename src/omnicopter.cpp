#include <ros/ros.h>
#include <Eigen/Eigen>
#include <omnicopter_sim/MotorCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

#include <tf/transform_broadcaster.h>

#define LOOP_RATE 600
#define IMU_RATE_FACTOR 6 //Imu rate = LOOP_RATE/IMU_RATE_FACTOR
#define K_F 4.2e-7
#define K_M 4.2e-8 //wild guess
#define max_rpm 29000
#define FREEZE_POSITION true
#define FREEZE_ROTATION false
#define DISABLE_GRAVITY false

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

const Eigen::Matrix3d J = 0.008*Eigen::Matrix3d::Identity(); //Inertia matrix
const double m = 0.693; //mass [kg]
const Eigen::Vector3d g(0,0,9.81);

visualization_msgs::MarkerArray visuals;

Eigen::MatrixXd X(3,8);
Eigen::MatrixXd P(3,8);

ros::Publisher pose_pub, imu_pub;


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
		double max_w = max_rpm/60.0*2*M_PI;
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
		double max_w = max_rpm/60.0*2*M_PI;
		double w = max_w*(pwm-1500)/500;
		if(w < 0){
			return -K_M*w*w;
		}
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
	return P_cross_X*getMotorForces();// + X*getMotorTorques();
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
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);
	

	ros::Subscriber motor_cmd_sub = nh.subscribe("motor_commands", 1, commandCallback); 


	double a = 0.5 + 1.0/sqrt(12);
	double b = 0.5- 1.0/sqrt(12);
	double c = 1.0/sqrt(3);
	X << -a, b,-b, a, a,-b, b,-a,
		  b, a,-a,-b,-b,-a, a, b,
		  c,-c,-c, c, c,-c,-c, c;

	P << 1,-1, 1,-1, 1,-1, 1,-1,
		 1, 1,-1,-1, 1, 1,-1,-1,
		 1, 1, 1, 1,-1,-1,-1,-1;

	P *= 0.12/sqrt(3); //Motors are placed 12cm from the center along the tubes => 13/sqrt(3) in x,y,z direction


	ros::Rate loop_rate(LOOP_RATE);
	double dt = 1.0/LOOP_RATE;
	Eigen::Vector3d f(0,0,0); //total force in BODY frame
	Eigen::Vector3d t(0,0,0); //total torque in BODY frame
	Eigen::Vector3d acc(0,0,0); //linear acceleration
	Eigen::Vector3d alpha(0,0,0); //angular acceleration
	unsigned int imu_msg_counter = 0;

	while(ros::ok()) {

		ros::spinOnce();

		Eigen::Matrix3d R = q.normalized().toRotationMatrix();
		f = getBodyForce();
		t = getBodyTorque();

		// Kinetics
		if(DISABLE_GRAVITY){
			acc = (1/m)*R*f;
		} else {
			acc = (1/m)*R*f - g;
		}
		
		alpha = J.inverse()*(t-w.cross(J*w));
		//ROS_INFO("alpha = %f %f %f",alpha(0), alpha(1), alpha(2));

		// Kinematics
		// Linear
		if(!FREEZE_POSITION){
			v += acc*dt;
			p += v*dt;	
		}
		

		// Angular
		if(!FREEZE_ROTATION){
			w += alpha*dt; 
			Eigen::Quaterniond w_q(0,w(0)*0.5,w(1)*0.5,w(2)*0.5);
			Eigen::Quaterniond q_dot = q*w_q;
			q.x() += q_dot.x()*dt;
			q.y() += q_dot.y()*dt;
			q.z() += q_dot.z()*dt;
			q.w() += q_dot.w()*dt;
			q.normalize();
		}
		//ROS_INFO("a(%.1f %.1f %.1f) alp(%.1f %.1f %.1f) w(%.1f %.1f %.1f) v(%.1f %.1f %.1f)", acc(0), acc(1), acc(2), alpha(0), alpha(1), alpha(2), w(0), w(1), w(2), v(0), v(1), v(2));


		publishPose();


		imu_msg_counter++;
		if(imu_msg_counter >= IMU_RATE_FACTOR){
			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = "body";
			imu_msg.orientation.x = q.x();
			imu_msg.orientation.y = q.y();
			imu_msg.orientation.z = q.z();
			imu_msg.orientation.w = q.w();
			imu_msg.angular_velocity.x = w(0);
			imu_msg.angular_velocity.y = w(1);
			imu_msg.angular_velocity.z = w(2);
			imu_msg.linear_acceleration.x = acc(0);
			imu_msg.linear_acceleration.y = acc(1);
			imu_msg.linear_acceleration.z = acc(2);
			imu_pub.publish(imu_msg);

			imu_msg_counter = 0;
		}


		
		static tf::TransformBroadcaster br;
		tf::Transform transform;
  		transform.setOrigin( tf::Vector3(p(0), p(1), p(2)) );
  		tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  		transform.setRotation(tf_q);
  		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));

		loop_rate.sleep();
	}

}