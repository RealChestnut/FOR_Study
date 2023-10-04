#include "cad_uav_util_function.hpp"


////ROS NODE HANDLE////

///////////////////////////////GENERAL PARAMETER START////////////////////////////////////////////

//Timer class :: c++11
std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;
std_msgs::Float32 dt;

//////////Global : XYZ  Body : xyz///////////////////

geometry_msgs::Vector3 tau_rpy_desired; // desired torque (N.m)

double tau_y_sin = 0; //yaw sine term torque (N.m)
double tau_y_d_non_sat=0;//yaw deried torque non-saturation (N.m)
double tau_y_th = 0; // yaw desired torque w.r.t. servo tilt (N.m)

geometry_msgs::Vector3 rpy_ddot_desired; // angular acceleration

double Thrust_d = 0;//altitude desired thrust(N)

geometry_msgs::Vector3 rpy_desired; // desired rpy angle

double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

geometry_msgs::Vector3 XYZ_desired; //desired global position
geometry_msgs::Vector3 XYZ_desired_base; // initial desired XYZ global position
geometry_msgs::Vector3 XYZ_dot_desired; // desired global linear velocity
geometry_msgs::Vector3 XYZ_ddot_desired; // desired global linear acceleration

geometry_msgs::Vector3 F_xyzd; // desired body force
geometry_msgs::Vector3 F_xyzd_main;
geometry_msgs::Vector3 F_xyzd_sub1;
geometry_msgs::Vector3 F_xyzd_sub2;

double r_arm = 0.3025;// m // diagonal length between thruster x2
double l_servo = 0.035; // length from servo motor to propeller

double mass_system =0.0; // system mass (kg) ex) M = main+sub1+sub2
double mass_main = 9.0; // main drone mass(kg) 
double mass_sub1 = 9.0; // sub1 drone mass (kg)
double mass_sub2 = 9.0; // sub2 drone mass (kg)

double r2=sqrt(2); // root(2)
double l_module = 0.50; //(m) module horizontal body length
double xi = 0.01;// aerodynamics constant value F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
double pi = 3.141592;//(rad)
double g = 9.80665;// gravity acceleration (m/s^2)

///////// Moment of Inertia ///////


///// Original MoI /////
double Jxx = 1.23;//6.75;
double Jyy = 1.23;//1.71;
double Jzz = 1.50;//5.53;
/////////////////////////////

//////// limitation value ////////
double rp_limit = 0.25;// roll pitch angle limit (rad)
double y_vel_limit = 0.01;// yaw angle velocity limit (rad/s)
double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
double T_limit = 80;// thrust limit (N)
double altitude_limit = 1;// z direction limit (m)
double XY_limit = 1.0; // position limit
double XYZ_dot_limit=1; // linear velocity limit
double XYZ_ddot_limit=2; // linear acceleration limit
double hardware_servo_limit=0.3; // servo limit w.r.t. hardware
double servo_command_limit = 0.3; // servo limit w.r.t. command part
double tau_y_limit = 1.0; // yaw torque limit
double tau_y_th_limit = 0.5; // yaw torque w.r.t. servo tilt limit

double F_xd_limit = mass_system*2.0; // X direction force limit 
double F_yd_limit = mass_system*2.0; // Y direction force limit

geometry_msgs::Vector3 CoM_hat; // center of mass

//////// CONTROL GAIN PARAMETER /////////

//integratior(PID) limitation
double integ_limit=10;
double integ_yaw_limit = 10;
double z_integ_limit=100;
double position_integ_limit=10;
double velocity_integ_limit=10;

//Roll, Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;

//Roll PID gains
double Par=3.5;
double Iar=0.4;
double Dar=0.5;

//Pitch PID gains
double Pap=0.0;
double Iap=0.0;
double Dap=0.0;

//Yaw PID gains
double Py=2.0;
double Iy=0.1;
double Dy=0.1;

//Z Velocity PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//XY Velocity PID gains
double Pv=5.0;
double Iv=0.1;
double Dv=5.0;

//Position PID gains
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;

//Conventional Flight Mode Control Gains

double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Par, tilt_Iar, tilt_Dar;
double tilt_Pap, tilt_Iap, tilt_Dap;

double tilt_Py, tilt_Iy, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;

//error data for PID controller

double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_y_i = 0;//yaw error integration
double e_X_i = 0;//X position error integration
double e_Y_i = 0;//Y position error integration
double e_Z_i = 0;//Z position error integration
double e_X_dot_i = 0;//X velocity error integration
double e_Y_dot_i = 0;//Y velocity error integration
double e_Z_dot_i = 0;//Z velocity error integration

//SBUS DATA CALLBACK//
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;


//////////////////////// TOPIC MESSAGE START //////////////////////


//////////////////////// SUBSCRIBER START /////////////////////////

ros::Subscriber dynamixel_state; // servo angle data callback
ros::Subscriber att; // imu data callback
ros::Subscriber rc_in; //Sbus signal callback from Arduino
ros::Subscriber battery_checker; // battery level callback from Arduino 
ros::Subscriber t265_position; // position data callback from T265 
ros::Subscriber t265_rotation; // angle data callback from T265
ros::Subscriber t265_odom; // odometry data (linear velocity) callback from T265


//////////////////////// PUBLISHER START /////////////////////////

ros::Publisher PWMs; // PWM data logging
ros::Publisher PWM_generator; // To ros-pwm-generator node

ros::Publisher goal_dynamixel_position; // To dynamixel position && servo data logging

ros::Publisher desired_motor_thrust; 

ros::Publisher desired_force; 
ros::Publisher desired_torque; 
ros::Publisher desired_torque_split;

ros::Publisher angular_Acceleration; 
ros::Publisher linear_acceleration; 

ros::Publisher linear_velocity;
ros::Publisher desired_velocity;
ros::Publisher angular_velocity;

ros::Publisher desired_position;
ros::Publisher position;

ros::Publisher euler; // euler angle data logging
ros::Publisher desired_angle; // desired angle data logging

ros::Publisher battery_voltage;
ros::Publisher delta_time;

ros::Publisher ToSubAgent;

