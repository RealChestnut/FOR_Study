//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application

#include <cad_uav_sub_util.hpp>
#include <vector>


//Function Operator--------------------------------------------------------------------



void rpyT_ctrl();
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void jointstateCallback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void setCM();
void setCM_Xc_p2();
void setSA();
void setMoI();//23_08_01
void publisherSet();
int32_t pwmMapping(double pwm);
void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4);
void pwm_Kill();
void pwm_Max();
void pwm_Arm();
void pwm_Calibration();
void pid_Gain_Setting();
void disturbance_Observer();
void sine_wave_vibration();
void get_Rotation_matrix();
void external_force_estimation();
void admittance_controller();
void position_dob();
//23.08.03---------------------------------------------

float Arr[12];

void shape_selector(int num);
void force_distributor();
void K_matrix();
void dataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

//-----------------------------------------------------


int main(int argc, char **argv){
	
    	ros::init(argc, argv,"cad_uav_sub_ctrl");

    	std::string deviceName;
    	ros::NodeHandle params("~");
    	params.param<std::string>("device", deviceName, "/gx5");

    	ros::NodeHandle nh;

	//Loading gains from the "t3_mav_controller.launch" file
		//integratior(PID) limitation
		integ_limit=nh.param<double>("attitude_integ_limit",10);
		z_integ_limit=nh.param<double>("altitude_integ_limit",100);
		pos_integ_limit=nh.param<double>("position_integ_limit",10);

		//Center of Mass
		x_c_hat=nh.param<double>("x_center_of_mass",1.0);
		y_c_hat=nh.param<double>("y_center_of_mass",1.0);
		z_c_hat=nh.param<double>("z_center_of_mass",1.0);
		CoM.x = x_c_hat;
		CoM.y = y_c_hat;
		CoM.z = z_c_hat;

		//Conventional Flight Mode Control Gains
			//Roll, Pitch PID gains
			conv_Pa=nh.param<double>("conv_attitude_rp_P_gain",3.5);
			conv_Ia=nh.param<double>("conv_attitude_rp_I_gain",0.4);
			conv_Da=nh.param<double>("conv_attitude_rp_D_gain",0.5);

			//Yaw PID gains
			conv_Py=nh.param<double>("conv_attitude_y_P_gain",2.0);
			conv_Dy=nh.param<double>("conv_attitude_y_D_gain",0.1);

			//Altitude PID gains
			conv_Pz=nh.param<double>("conv_altitude_P_gain",16.0);
			conv_Iz=nh.param<double>("conv_altitude_I_gain",5.0);
			conv_Dz=nh.param<double>("conv_altitude_D_gain",15.0);

			//Velocity PID gains
			conv_Pv=nh.param<double>("conv_velocity_P_gain",5.0);
			conv_Iv=nh.param<double>("conv_velocity_I_gain",1.0);
			conv_Dv=nh.param<double>("conv_velocity_D_gain",5.0);

			//Position PID gains
			conv_Pp=nh.param<double>("conv_position_P_gain",3.0);
			conv_Ip=nh.param<double>("conv_position_I_gain",0.1);
			conv_Dp=nh.param<double>("conv_position_D_gain",5.0);

		//Tilt Flight Mode Control Gains
			//Roll, Pitch PID gains
			
			tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
                        tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
                        tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);

			//Yaw PID gains
			tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
			tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

			//Altitude PID gains
			tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
			tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
			tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

			//Velocity PID gains
			tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
			tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
			tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

			//Position PID gains
			tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
			tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
			tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);

	//----------------------------------------------------------
	//Initialization parameter matrix & vector for Control -----
		p_c_main << 0, 0, 0;
		p_c_sub1 << 0, 0, 0;
		p_c_sub2 << 0, 0, 0;
		setMoI();
		//setCM(); //Control matrix
        	setCM_Xc_p2(); // Control matrix w.r.t. Xc_p2
		K_matrix();
		F_cmd << 0, 0, 0, 0; //force command
		rpy_ddot_d << 0, 0, 0; // angular_acceleration desired
		tau_cmd << 0, 0, 0; //torque command
		Dump << 0,0,0,1; // for allocation factor calculation
		send_sub_data.data.resize(12);
	//----------------------------------------------------------
	//
    

    PWMs_sub = nh.advertise<std_msgs::Int16MultiArray>("PWMs_sub", 1); // PWM 1,2,3,4
    PWM_generator_sub = nh.advertise<std_msgs::Int32MultiArray>("command_sub",1);  // publish to pca9685
    goal_dynamixel_position_sub  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position_sub",100); // desired theta1,2
	Forces_sub = nh.advertise<std_msgs::Float32MultiArray>("Forces_sub",100); // F 1,2,3,4
	battery_voltage_sub = nh.advertise<std_msgs::Float32>("battery_voltage_sub",100);
	//ros::Subscriber rc_in = nh.subscribe("/sbus_sub",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber dynamixel_state_sub = nh.subscribe("joint_states_sub",100,jointstateCallback,ros::TransportHints().tcpNoDelay());

	ros::Subscriber battery_checker = nh.subscribe("/battery_sub",100,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber master2slave_data = nh.subscribe("master2slave_data_pub",1,dataCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

void publisherSet(){
	

	//-parameter changer w.r.t. module shape-//
	
	//description//
	//if module combined left or right in 2 modules flight case
	// if
	// case 1 :: 1drone - 
	// case 2 :: 2drone --
	// case 3 :: 2drone : 
	// case 4 :: 3drone --- 
	// case 5 :: 3drone |
	// case 6 :: 3drone r
	// case 7 :: 3drone L
		
	
	//--Timer Update-------------------------------//

	end=std::chrono::high_resolution_clock::now();
	delta_t=end-start; // use for derivation 
	dt.data=delta_t.count();
	start=std::chrono::high_resolution_clock::now();
	//-------------------------------------------//
	
	shape_selector(2);
	
	setMoI();
		
	/*
	ROS_INFO("%f|%f|%f, %f|%f|%f, %f|%f|%f",
	hat_MoI(0,0),hat_MoI(0,1),hat_MoI(0,2),
	hat_MoI(1,0),hat_MoI(1,1),hat_MoI(1,2),
	hat_MoI(2,0),hat_MoI(2,1),hat_MoI(2,2));*/
	// Control Allocation Matrix update	
	//setCM();
	setCM_Xc_p2();
	
	//------------------------------
	
	// Inertia Tensor update 23.08.01
	//------------------------------
	
	//angular_accerlation update-----------------------------------------
	angular_Accel.x = (imu_ang_vel.x-prev_angular_Vel.x)/delta_t.count();
	angular_Accel.y = (imu_ang_vel.y-prev_angular_Vel.y)/delta_t.count();
	angular_Accel.z = (imu_ang_vel.z-prev_angular_Vel.z)/delta_t.count();
	//-------------------------------------------------------------------
	
	
	if(!position_mode){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		X_r = X_d;
		Y_r = Y_d;
		e_X_i=0;
		e_Y_i=0;
		if(attitude_mode){
			e_X_dot_i=0;
			e_Y_dot_i=0;	
		}	
	}

	if(kill_mode){	
		y_d=imu_rpy.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
		theta1_command=0.0;
		theta2_command=0.0;
		theta3_command=0.0;
		theta4_command=0.0;
		pwm_Kill();
		//ROS_INFO("Kill mode");	
	}
	else{

		rpyT_ctrl();
	//	pwm_Arm();		

		//ROS_INFO("Arm mode");	
	}

//	pwm_Calibration();	
	Force.data.resize(4);
	Force.data[0] = F1;
	Force.data[1] = F2;
	Force.data[2] = F3;
	Force.data[3] = F4;
	Forces_sub.publish(Force);// force conclusion
	
	PWMs_sub.publish(PWMs_cmd);// PWMs_d value
	goal_dynamixel_position_sub.publish(servo_msg_create(theta3_command,theta4_command, theta1_command, theta2_command)); // desired theta
	PWM_generator_sub.publish(PWMs_val);
	battery_voltage_sub.publish(battery_voltage_msg);
//-------------------------------------------------------------------------//
}


void setCM(){
	//Co-rotating type //2023_07_27 update
	CM << (y_c_hat+r_arm/r2)*cos(theta1)+(-(l_servo-z_c_hat)+xi)*sin(theta1)/r2,  (y_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)-xi)*sin(theta2)/r2,   (y_c_hat-r_arm/r2)*cos(theta3)+((l_servo-z_c_hat)-xi)*sin(theta3)/r2,  (y_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)+xi)*sin(theta4)/r2,
	      -(x_c_hat-r_arm/r2)*cos(theta1)+((l_servo-z_c_hat)+xi)*sin(theta1)/r2, -(x_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)+xi)*sin(theta2)/r2, -(x_c_hat+r_arm/r2)*cos(theta3)+(-(l_servo-z_c_hat)-xi)*sin(theta3)/r2, -(x_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)-xi)*sin(theta4)/r2,
	      -xi*cos(theta1)+(r_arm-(x_c_hat-y_c_hat)/r2)*sin(theta1) 			   ,   xi*cos(theta2)+(r_arm+(x_c_hat+y_c_hat)/r2)*sin(theta2)			  ,   -xi*cos(theta3)+(r_arm+(x_c_hat-y_c_hat)/r2)*sin(theta3)			  ,   xi*cos(theta4)+(r_arm-(x_c_hat+y_c_hat)/r2)*sin(theta4),
																   -cos(theta1),                                                          -cos(theta2),                                                           -cos(theta3),                                                           -cos(theta4);
    invCM = CM.inverse();
//	      -xi*cos(theta1)+(y_c_hat-x_c_hat+r2*r_arm)*sin(theta1)/r2,  xi*cos(theta2)+(x_c_hat+y_c_hat+r2*r_arm)*sin(theta2)/r2,  -xi*cos(theta3)+(x_c_hat-y_c_hat+r2*r_arm)*sin(theta3)/r2,  xi*cos(theta4)+(-x_c_hat-y_c_hat+r2*r_arm)*sin(theta4)/r2,
}

void setCM_Xc_p2(){

	CM_Xc_p2 << (X_c_p2(1)+r_arm/r2)*cos(theta1)+(-(l_servo-X_c_p2(2))+xi)*sin(theta1)/r2,  (X_c_p2(1)+r_arm/r2)*cos(theta2)+((l_servo-X_c_p2(2))-xi)*sin(theta2)/r2,   (X_c_p2(1)-r_arm/r2)*cos(theta3)+((l_servo-X_c_p2(2))-xi)*sin(theta3)/r2,  (X_c_p2(1)-r_arm/r2)*cos(theta4)+(-(l_servo-X_c_p2(2))+xi)*sin(theta4)/r2,
	      -(X_c_p2(0)-r_arm/r2)*cos(theta1)+((l_servo-X_c_p2(2))+xi)*sin(theta1)/r2, -(X_c_p2(0)+r_arm/r2)*cos(theta2)+((l_servo-X_c_p2(2))+xi)*sin(theta2)/r2, -(X_c_p2(0)+r_arm/r2)*cos(theta3)+(-(l_servo-X_c_p2(2))-xi)*sin(theta3)/r2, -(X_c_p2(0)-r_arm/r2)*cos(theta4)+(-(l_servo-X_c_p2(2))-xi)*sin(theta4)/r2,
	      -xi*cos(theta1)+(r_arm-(X_c_p2(0)-X_c_p2(1))/r2)*sin(theta1) 			   ,   xi*cos(theta2)+(r_arm+(X_c_p2(0)+X_c_p2(1))/r2)*sin(theta2)			  ,   -xi*cos(theta3)+(r_arm+(X_c_p2(0)-X_c_p2(1))/r2)*sin(theta3)			  ,   xi*cos(theta4)+(r_arm-(X_c_p2(0)+X_c_p2(1))/r2)*sin(theta4),
																   -cos(theta1),                                                          -cos(theta2),                                                           -cos(theta3),                                                           -cos(theta4);
	invCM_Xc_p2 = CM_Xc_p2.inverse();
}

void setSA(){

	SA <<  F1/r2,     F2/r2,     -(F3/r2),     -(F4/r2),
	       F1/r2,    -(F2/r2),   -(F3/r2),       F4/r2,
	      r_arm*F1,  r_arm*F2,   r_arm*F3,   r_arm*F4,
	      r_arm*F1, -(r_arm*F2),   r_arm*F3,  -(r_arm*F4);

	invSA = SA.inverse();

}
void rpyT_ctrl() {


	pid_Gain_Setting();
	/*
	rpy_ddot_d(0) = Par* e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x);//- (double)0.48;
	rpy_ddot_d(1) = Par * e_p + Iar * e_p_i + Dar * (-imu_ang_vel.y);//+ (double)0.18; 
	rpy_ddot_d(2) = Py * e_y + Dy * (-imu_ang_vel.z);
	
	angular_acc_d.x = rpy_ddot_d(0);//r_ddot_d
	angular_acc_d.y = rpy_ddot_d(1);//p_ddot_d
	angular_acc_d.z = rpy_ddot_d(2);//y_ddot_d
	*/
	//tau_cmd = (hat_MoI*rpy_ddot_d)/module_num_count; //torque distributor
	//ROS_INFO("%f | %f | %f",tau_cmd(0),tau_cmd(1),tau_cmd(2));
	
	
	
	tau_r_d = tau_cmd(0);
	tau_p_d = tau_cmd(1);
	tau_y_d = tau_cmd(2);


	K_matrix(); // For force allocation :: 23.08.17
	
	
	torque_d.x = tau_r_d;
	torque_d.y = tau_p_d;
	torque_d.z = tau_y_d;
	force_d.x = F_xd;
	force_d.y = F_yd;
	force_d.z = F_zd;
	
	ROS_INFO("%f %f %f", F_xd,F_yd,F_zd);	
	u << tau_r_d, tau_p_d, tau_y_d, F_zd;
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d); //but not use 22.10.12
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	F_cmd = invCM_Xc_p2*u;
	//F_cmd = invCM*u;
	F1 = F_cmd(0);
	F2 = F_cmd(1);
	F3 = F_cmd(2);
	F4 = F_cmd(3);

	//tau_yaw_sine_desired part---//
	if((tau_y_d-tau_y_limit)==0)
	{
	tau_y_th = tau_y_d_non_sat-tau_y_d;
	if(fabs(tau_y_th) > tau_y_th_limit) tau_y_th = (tau_y_th/fabs(tau_y_th))*tau_y_th_limit;//2023.08.17 update
	}

	//----------------------------//
	control_by_theta << F_xd, F_yd, tau_y_th, 0;
	
	setSA();
	sine_theta_command = invSA*control_by_theta; //2023.08.05 update
	if(!tilt_mode){
		theta1_command = 0.0;
        	theta2_command = 0.0;
		theta3_command = 0.0;
		theta4_command = 0.0;
	}
	//Tilting type
	else {
		theta1_command = asin(sine_theta_command(0));
		theta2_command = asin(sine_theta_command(1));
		theta3_command = asin(sine_theta_command(2));
		theta4_command = asin(sine_theta_command(3));
 		if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
		if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
		if(fabs(theta3_command)>hardware_servo_limit) theta3_command = (theta3_command/fabs(theta3_command))*hardware_servo_limit;
		if(fabs(theta4_command)>hardware_servo_limit) theta4_command = (theta4_command/fabs(theta4_command))*hardware_servo_limit;
	}

	//pwm_Kill();
	pwm_Command(Force_to_PWM(F3),Force_to_PWM(F4), Force_to_PWM(F1), Force_to_PWM(F2));
}

void dataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        for(int i=0;i<12;i++){
                Arr[i]=msg->data[i];
        }

	F_xd=Arr[0];
	F_yd=Arr[1];
	F_zd=Arr[2];
	tau_cmd(0)=Arr[3];
	tau_cmd(0)=Arr[4];
	tau_cmd(0)=Arr[5];
	kill_mode = static_cast<bool>(Arr[6]);
	altitude_mode =static_cast<bool>( Arr[7]);
	attitude_mode = static_cast<bool>(Arr[8]);
	velocity_mode = static_cast<bool>(Arr[9]);
	position_mode = static_cast<bool>(Arr[10]);
	tilt_mode = static_cast<bool>(Arr[11]);

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
	//Force=A*pwm^2+B*pwm+C
	// A = 0.00004 / B = -0.0568 / C = 17.546 

	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3)/param4;
		// ROS_INFO("%lf",pwm);
	}
	else pwm = 1100.;
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	/*if(altitude_mode){//altitude mode
		if(Z_d_base<=0){
			if(Z_d>Z_d_base && !start_flag) {
				pwm=1100;
			}
			else if(Z_d<Z_d_base) start_flag=true;
		}
		else pwm=param1;
	}*/
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    	rac_servo_value=msg;
	theta1=msg.position[2];
	theta2=msg.position[3];
	theta3=msg.position[0];
	theta4=msg.position[1];
	
    	//ROS_INFO("theta1:%lf   theta2:%lf",theta1, theta2);
}

ros::Time imuTimer;

void imu_Callback(const sensor_msgs::Imu& msg){
    	imu=msg;
    
    	// TP attitude - Quaternion representation
    	imu_quaternion=msg.orientation;
    	imu_ang_vel=msg.angular_velocity;
    	// ROS_INFO("R:%lf\tP:%lf\tY:%lf",imu_ang_vel.x,imu_ang_vel.y,imu_ang_vel.z);
    	imu_lin_acc=msg.linear_acceleration;

    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(imu_quaternion,quat);

    	// TP attitude - Euler representation
    	tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
	base_yaw = cam_att(2);
    	if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	yaw_now = base_yaw+2*pi*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	//imuTimer = ros::Time::now();
}

void filterCallback(const sensor_msgs::Imu& msg){
	filtered_angular_rate=msg.angular_velocity;
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

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
	
	if(Sbus[5]>1500) altitude_mode=true;
	else altitude_mode=false;

	if(Sbus[6]<1300){
		attitude_mode=true;
		velocity_mode=false;
		position_mode=false;
	}
	else if(Sbus[6]<1700){
		attitude_mode=false;
		velocity_mode=true;
		position_mode=false;
	}
	else{
		attitude_mode=false;
		velocity_mode=false;
		position_mode=true;
	}

	if(Sbus[7]>1500) tilt_mode=true;
	else tilt_mode=false;

	if(Sbus[9]>1500) admittance_mode=true;
	else admittance_mode=true;

	
//	ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d",Sbus[0],Sbus[1],Sbus[2],Sbus[3],Sbus[4],Sbus[5],Sbus[6],Sbus[7]);
	//if(Sbus[9]>1500) ESC_control=true;
	//else ESC_control=false;
}


void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*5.0/(double)1024/(7440./(30000.+7440.)); //4096
	double kv=0.08;
	voltage=kv*voltage+(1-kv)*voltage_old;
	voltage_old=voltage;
	//ROS_INFO("%f",voltage);
	if(voltage>25.2) voltage=25.2;
	if(voltage<20.0) voltage=20.0;
	battery_voltage_msg.data=voltage;
	
}

ros::Time posTimer;
void posCallback(const geometry_msgs::Vector3& msg){

	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=msg.z;
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
	//ROS_INFO("%f, %f, %f",t265_att.x,t265_att.y,t265_att.z);	
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;

	R_v << 0, -r2/2, r2/2,
	    	0, -r2/2, -r2/2,
		1, 0, 0;

	v = R_v*cam_v;

	double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
	double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
	double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

	lin_vel.x=global_X_dot;
	lin_vel.y=global_Y_dot;
	lin_vel.z=global_Z_dot;
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Rotate Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",cam_v(0),cam_v(1),cam_v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}

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
void pwm_Calibration(){
	//if(Sbus[4]>1500) pwm_Arm();
	//else pwm_Kill();
        if(Sbus[6]>1700) pwm_Kill();
	if(Sbus[6]<1000) pwm_Arm();
//ROS_INFO("Sbus[6] : %d", Sbus[6]);	
}


void pid_Gain_Setting(){
	if(Sbus[7]<=1500){
		Par = conv_Pa;
		Iar = conv_Ia;
		Dar = conv_Da;

		Py = conv_Py;
		Dy = conv_Dy;

		Pz = conv_Pz;
		Iz = conv_Iz;
		Dz = conv_Dz;
		
		Pv = conv_Pv;
		Iv = conv_Iv;
		Dv = conv_Dv;

		Pp = conv_Pp;
		Ip = conv_Ip;
		Dp = conv_Dp;
	}
	else{
		Par = tilt_Par;
		Iar = tilt_Iar;
		Dar = tilt_Dar;

		Py = tilt_Py;
		Dy = tilt_Dy;

		Pz = tilt_Pz;
		Iz = tilt_Iz;
		Dz = tilt_Dz;
		
		Pv = tilt_Pv;
		Iv = tilt_Iv;
		Dv = tilt_Dv;

		Pp = tilt_Pp;
		Ip = tilt_Ip;
		Dp = tilt_Dp;
	}
	//ROS_INFO("%.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf ",Pa, Ia, Da, Py, Dy, Pz, Iz, Dz, Pp, Ip, Dp);
}




