#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "nav_msgs/Odometry.h"


std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;

double pwm_freq=417.3;//pwm signal frequency

int32_t pwmMapping(double pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));
}

void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = pwm1;
	PWMs_cmd.data[1] = pwm2;
	PWMs_cmd.data[2] = pwm3;
	PWMs_cmd.data[3] = pwm4;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm1);
	PWMs_val.data[1] = pwmMapping(pwm2);
	PWMs_val.data[2] = pwmMapping(pwm3);
	PWMs_val.data[3] = pwmMapping(pwm4);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Max(){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;
}

void pwm_Kill(){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1000;
	PWMs_cmd.data[1] = 1000;
	PWMs_cmd.data[2] = 1000;
	PWMs_cmd.data[3] = 1000;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Arm(){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1500;
	PWMs_cmd.data[1] = 1500;
	PWMs_cmd.data[2] = 1500;
	PWMs_cmd.data[3] = 1500;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1500.);
	PWMs_val.data[1] = pwmMapping(1500.);
	PWMs_val.data[2] = pwmMapping(1500.);
	PWMs_val.data[3] = pwmMapping(1500.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

double Force_to_PWM(double F) {
	double pwm;
	//double A = -9.8*pow(10.0,-8.0)*pow(voltage,2.0)+3.23*pow(10.0,-6.0)*voltage-1.8*pow(10.0,-5.0);
	//double B = 0.000243*pow(voltage,2.0)-0.00663*voltage+0.03723;
	//double C = -0.11063*pow(voltage,2.0)+2.332691*voltage-10.885;
	double param1 = 710;//-B/(2.0*A);
	double param2 = 0.00016;//1.0/A;
	double param3 = 0.00041888;//(pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	double param4 = 0.00008;

	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3)/param4;

	}
	else pwm = 1100.;
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	return pwm;
}

sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4){
	sensor_msgs::JointState servo_msg;

	servo_msg.header.stamp=ros::Time::now();

	servo_msg.name.resize(4);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";
	servo_msg.name[2]="id_3";
	servo_msg.name[3]="id_4";


	servo_msg.position.resize(4);
	servo_msg.position[0]=desired_theta1;
	servo_msg.position[1]=desired_theta2;
	servo_msg.position[2]=desired_theta3;
	servo_msg.position[3]=desired_theta4;
	//ROS_INFO("rr: %lf, rp: %lf",rr,rp);
	return servo_msg;
}
