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

//////////////////////// TOPIC MESSAGE START //////////////////////


//////////////////////// SUBSCRIBER START /////////////////////////

ros::Subscriber dynamixel_state; // servo angle data callback
ros::Subscriber att; // imu data callback
ros::Subscriber rc_in; //Sbus signal callback from Arduino
ros::Subscriber battery_checker; // battery level callback from Arduino 
ros::Subscriber Switch_checker; // switch on off data callback from Arduino

ros::Subscriber t265_position; // position data callback from T265 
ros::Subscriber t265_rotation; // angle data callback from T265
ros::Subscriber t265_odom; // odometry data (linear velocity) callback from T265

ros::Subscriber main2sub_data; // main 2 sub data callback

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

///////////////////////////////CALLBACK FUNCTION START////////////////////////////////////////////

void Clock()
{
  end=std::chrono::high_resolution_clock::now();
  delta_t=end-start; 
  start=std::chrono::high_resolution_clock::now();
  dt.data=delta_t.count();
  //ROS_INFO_STREAM("Clock Time : " << dt.data);

}

//SERVO ANGLE CALLBACK//
Eigen::VectorXd servo_theta(5);
void jointstate_Callback(const sensor_msgs::JointState& msg)
{
    servo_theta(0)=msg.position[0];
    servo_theta(1)=msg.position[1];
    servo_theta(2)=msg.position[2];
    servo_theta(3)=msg.position[3];
    servo_theta(4)=msg.position[4];
}


  

//IMU DATA CALLBACK//
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acl;
geometry_msgs::Vector3 prev_ang_vel;
Eigen::Matrix3d W2B_rot;
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;
double t265_yaw_angle=0;
void imu_Callback(const sensor_msgs::Imu& msg)
{
    // TP attitude - Quaternion representation
    imu_quaternion=msg.orientation;
    imu_ang_vel=msg.angular_velocity;
    imu_lin_acl=msg.linear_acceleration;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_quaternion,quat);

    // TP attitude - Euler representation
    tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
    base_yaw = t265_yaw_angle;
    if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
    else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	  yaw_now = base_yaw+2*pi*yaw_rotate_count;
	
    imu_rpy.z = yaw_now;
    yaw_prev = base_yaw;

    W2B_rot <<  
    cos(imu_rpy.y)*cos(imu_rpy.z), 
    cos(imu_rpy.y)*sin(imu_rpy.z), 
    sin(imu_rpy.y), 
    cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.y)*sin(imu_rpy.x)*sin(imu_rpy.z), 
    cos(imu_rpy.y)*sin(imu_rpy.x), 
    cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y)+sin(imu_rpy.x)*sin(imu_rpy.z), 
    cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z), 
    cos(imu_rpy.y)*cos(imu_rpy.x);
	
}



geometry_msgs::Quaternion rot;
void t265_rotation_Callback(const geometry_msgs::Quaternion& msg)
{
  rot.x=msg.x;
  rot.y=msg.y;
  rot.z=msg.z;
  rot.w=msg.w;

}


//LINEAR VELOCITY DATA CALLBACK//
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
geometry_msgs::Vector3 lin_vel_from_t265;
geometry_msgs::Vector3 ang_vel_from_t265;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Quaternion t265_quat;

void t265_Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    lin_vel_from_t265=msg->twist.twist.linear;
    ang_vel_from_t265=msg->twist.twist.angular;
    t265_quat=msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(rot,quat);
    tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
    t265_yaw_angle = t265_att.z;
    cam_v << lin_vel_from_t265.x, lin_vel_from_t265.y, lin_vel_from_t265.z;  

    //rotation matrix for aligning from camera axis to body axis
    R_v <<
            0, -r2/2, r2/2,
            0, -r2/2, -r2/2,
            1, 0, 0;

    //camera axis to body axis
    v = R_v*cam_v; // linear velocity

    // body axis to global axis :: linear velocity
    double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
    double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
    double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

    lin_vel.x=global_X_dot;
    lin_vel.y=global_Y_dot;
    lin_vel.z=global_Z_dot;
}



//Accelerometer LPF//
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_az_dot = 0;
double x_ax = 0;
double x_ay = 0;
double x_az = 0;
double accel_cutoff_freq = 1.0;

geometry_msgs::Vector3 lin_acl;
void Accelerometer_LPF()
{
  x_ax_dot=-accel_cutoff_freq*x_ax+imu_lin_acl.x;
	x_ax+=x_ax_dot*delta_t.count();
	x_ay_dot=-accel_cutoff_freq*x_ay+imu_lin_acl.y;
	x_ay+=x_ay_dot*delta_t.count();
	x_az_dot=-accel_cutoff_freq*x_az+imu_lin_acl.z;
	x_az+=x_az_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	lin_acl.y=accel_cutoff_freq*x_ay;
	lin_acl.z=accel_cutoff_freq*x_az;
}


//SBUS DATA CALLBACK//
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;
int16_t Sbus[10];

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sbus_Callback(const std_msgs::Int16MultiArray::ConstPtr& array)
{
    for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
}

//POSITION DATA CALLBACK//
geometry_msgs::Vector3 position_from_t265; 
void t265_position_Callback(const geometry_msgs::Vector3& msg)
{
    position_from_t265.x=msg.x;
    position_from_t265.y=msg.y;
    position_from_t265.z=msg.z;
}
  


//BATTERY DATA CALLBACK//

double voltage=22.4;
double voltage_old=22.4;
std_msgs::Float32 battery_voltage_msg;
void battery_Callback(const std_msgs::Int16& msg)
{
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

// ARDUINO SWITCH DATA CALLBACK //
bool switch_toggle_from_ardu;
void switch_Callback(const std_msgs::UInt16& msg)
{
  switch_toggle_from_ardu=msg.data;
}

// wrench allocation data(torque, force) && kill_switch command from main //
Eigen::VectorXd wrench_allo_vector(6);
bool kill_from_main=false;
void main2sub_data_Callback(const std_msgs::Float32MultiArray& msg)
{
  
  wrench_allo_vector(0)=msg.data[0]; //Fx
  wrench_allo_vector(1)=msg.data[1]; //Fy
  wrench_allo_vector(2)=msg.data[2]; //Fz
  wrench_allo_vector(3)=msg.data[3]; //Tr
  wrench_allo_vector(4)=msg.data[4]; //Tp
  wrench_allo_vector(5)=msg.data[5]; //Ty
  kill_mode=static_cast<bool>(msg.data[6]);
}



///////////////////////////////CONSTROLLER START////////////////////////////////////////////

int module_num=1;
bool mono_flight = true;
int button_cnt=0;
bool main_agent=true;
bool sub_agent=false;
double servo_90rot=90.0;
void shape_detector()
{

  //main인지 sub인지 플레그
  //아두이노로 부터 switch 데이터 수신, 일정
  //눌린 스위치의 위치에 따라서 결합 형태 판단.
  //일정 조건을 만족했을때 결합됐다는 신호 날림.
  //하나씩 하나씩 결합될것임.
  //결합됐다고 판단이 되면은 여기서 서보모터에 회전 신호 인풋
  //
  //Eigen3

  /////// button data toggling ////////
  if(switch_toggle_from_ardu){
    if(button_cnt==0){
      button_cnt=1;} 
  }
  if(!switch_toggle_from_ardu){
    if(button_cnt!=0){
      button_cnt=0;}
  }
  ////////////////////////////////////
  if(button_cnt==1){
    servo_90rot=90.0;
  }
  if(button_cnt==0){
    servo_90rot=0.0;
  }

  

  module_num=1;

}

Eigen::Matrix3d origin_MoI;
Eigen::Matrix3d hat_CoM_x_main;
Eigen::Matrix3d hat_CoM_x_sub1;
Eigen::Matrix3d hat_CoM_x_sub2;
Eigen::Matrix3d hat_MoI;
int toggle_sub1=0;
int toggle_sub2=0;

void setMoI(){
	
	origin_MoI << 
  Jxx,   0,    0,
  0,   Jyy,    0,
  0,      0, Jzz;

	hat_CoM_x_main << 
           0,  -CoM_hat.z,    CoM_hat.y, 
   CoM_hat.z,           0,   -CoM_hat.x,   
  -CoM_hat.y,   CoM_hat.x,            0;

  hat_CoM_x_sub1 << 
           0,  -CoM_hat.z,    CoM_hat.y+2*l_module, 
   CoM_hat.z,           0,              -CoM_hat.x,   
  -(CoM_hat.y+2*l_module),   CoM_hat.x,          0;

  hat_CoM_x_sub2 << 
           0,  -CoM_hat.z,                CoM_hat.y, 
   CoM_hat.z,           0,  -(CoM_hat.x+2*l_module),   
  -CoM_hat.y,    CoM_hat.x-2*l_module,            0;



	
	hat_MoI = (origin_MoI  - mass_main*hat_CoM_x_main*hat_CoM_x_main)+
                        toggle_sub1*(origin_MoI  - mass_sub1*hat_CoM_x_sub1*hat_CoM_x_sub1)+
                        toggle_sub2*(origin_MoI  - mass_sub2*hat_CoM_x_sub2*hat_CoM_x_sub2);
        /*	
	ROS_INFO("%f|%f|%f, %f|%f|%f, %f|%f|%f",
	hat_MoI(0,0),hat_MoI(0,1),hat_MoI(0,2),
	hat_MoI(1,0),hat_MoI(1,1),hat_MoI(1,2),
	hat_MoI(2,0),hat_MoI(2,1),hat_MoI(2,2));*/
}

void pid_Gain_Setting()
{
	Par = tilt_Par;
	Iar = tilt_Iar;
	Dar = tilt_Dar;

	Pap = tilt_Pap;
  Iap = tilt_Iap;
  Dap = tilt_Dap;
	
	Py = tilt_Py;
	Iy = tilt_Iy;
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

Eigen::Vector3d X_c_p1;
Eigen::Vector3d X_c_p2;
void UpdateParameter()
{

  //shpe detector의 조건에 따라서 parameter 값 update
  //1. 전체 system mass
  //2. 전체 system COM
  //3. Xc_p1, Xc_p2 control logic에 들어갈 COM
  //4. 각종 limitation value :: ex) Fxyzd, thrust, lin_acl, lin_vel, torque_d, ect.

  // combination rule 
  // 1_ main drone must be located on the far left
  // 2_ main drone must be located at the bottom
  // 3_ sub1 drone must be battery exchanger	

  // setMoI value changed in here	
  // 전역에서 다른 전역변수와 연산으로 정의한 변수는
  // 업데이트 하고자 할 경우 함수 루프에서 한번더 돌려야함.
 

  int num = 1;

  if(num==1){
                    
                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0, 0, 0;
                //p_c_sub2 << 0, 0, 0;

                mass_system = mass_main;

                X_c_p1 << 0,0,0;
                X_c_p2 << CoM_hat.x,CoM_hat.y,CoM_hat.z;
                toggle_sub1=0;
                toggle_sub2=0;               

  }
  else if(num==2){
      

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << x_c_hat,y_c_hat+2*l_module,z_c_hat;
                //p_c_sub2 << 0,0,0;

                mass_system = mass_main+mass_sub1;

                X_c_p1 << 0, CoM_hat.y, 0;
                X_c_p2 << CoM_hat.x, 0, CoM_hat.z;
                toggle_sub1=1;
                toggle_sub2=0;

  }
  else if(num==3){

                //p_c_main << x_c_hat,y_c_hat,z_c_hat;
                //p_c_sub1 << 0,0,0;
                //p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

                mass_system = mass_main+mass_sub1+mass_sub2;
                X_c_p1 << CoM_hat.x,CoM_hat.y,CoM_hat.z;
                X_c_p2 << 0,0,0;
                toggle_sub1=1;
                toggle_sub2=1;
  }
  // Data (combinated with other data)
  F_xd_limit=mass_system*2;
  F_yd_limit=mass_system*2;
  //// system MoI Initialize ////
  pid_Gain_Setting();
  setMoI();  
  
}

//-----------------------GUI control parameter---------------------------//
double freq=200;//controller loop frequency

double Landing_time = 2.0; //put landing time (sec)
double Hovering_time = 1.0; //put Hovering time (sec)
double X_Speed = 0.2; //desired value change speed of x_axis
double Y_Speed = 0.2; //desired value change speed of y_axis
double yaw_Speed = 0.2; //desired value change speed of yaw
double Z_Speed = 0.2; //desired value change speed of z_axis
//---------------------------------------------------------//
double Landing_Inc = 1 / (delta_t.count() * Landing_time);
double Hovering_Inc = 1 / (delta_t.count() * Hovering_time);
double X_Inc = X_Speed / delta_t.count();
double Y_Inc = Y_Speed / delta_t.count();
double yaw_Inc = yaw_Speed / delta_t.count();
double z_Inc = Z_Speed / delta_t.count();
double X_Goal = 0;
double Y_Goal = 0;
double y_Goal = 0;
double z_Goal = 0;
//--------------------------------------------------------

//////Ground Station Boolian ////////
bool isKill = false; //ASDF
bool isArm = false;  //ASDF
bool isHover = false; //ASDF
bool isHovering = false; //ASDF
bool isLanding = false; //ASDF
bool isTilt = false;
int Hover_Checker = 0;
int Land_Checker = 0;

void Command_Generator()
{

  

  if(true)
  {
    /////////// angle command /////////////
    rpy_desired.x = sin(delta_t.count());//0.0;
    rpy_desired.y = sin(delta_t.count());//0.0;
    y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
    if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
    rpy_desired.z+=y_d_tangent;


    /////////// position command /////////////

    XYZ_desired.x = XYZ_desired_base.x - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
    XYZ_desired.y = XYZ_desired_base.y + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
    
    if(Sbus[2]>1800){
			XYZ_desired.z-=0.0005;
		}
		else if(Sbus[2]<1200){
			XYZ_desired.z+=0.0005;
		}
			
		if(XYZ_desired.z <-0.7) XYZ_desired.z=-0.7;
		if(XYZ_desired.z > 0) XYZ_desired.z=0;
  }
  
  if(false/*ground station*/)
  {
    ///////// //angle command ////////////
    
    rpy_desired.x = 0.0;
    rpy_desired.y = 0.0;

    ///////// Ground Station /////////////
    if(isLanding){
	    if(XYZ_desired.z>-0.1) XYZ_desired.z -= Landing_Inc; // 2.5초간 하강 ASDF
	    else{
		    if(position_from_t265.x<0.15){
			    Land_Checker ++;
			    if(Land_Checker>=100) //0.5초간 유지 ASDF
			    {
				    isLanding = false;
				    isHover = false; //ASDF
				    isArm = false; //ASDF
              //Service.request.FAC_isLanding = false;
              //Service.request.FAC_isHover = false;
              //Service.request.FAC_isHovering = false;
              //HoverClient.call(Service);
	      }
        }
        else Land_Checker = 0;		
      }
    }

    if(isHovering){
	    if(XYZ_desired.z<=1) XYZ_desired.z += Hovering_Inc; // 2.5초간 상승 ASDF
	    else{
		    //if(pos.z>0.9 && pos.z<1.1)  //1미터 +- 10cm 범위내에 도달하면
		    ////{
		    ////Hover_Checker ++;
		    ////if(Hover_Checker >= 600)
		    ////{
            	isHovering = false;
            	//Service.request.FAC_isHovering = false;
            	//Service.request.FAC_isHover = true;
            	//Service.request.FAC_isLanding = false;
            	//HoverClient.call(Service);
            	z_Goal = XYZ_desired.z;
          	//}
        	//}
        	//else Hover_Checker = 0;
		}
    }

    if (!isHovering || !isLanding )
    {
      if(X_Goal - XYZ_desired.x >= X_Inc ) XYZ_desired.x +=X_Inc;
      if(X_Goal - XYZ_desired.x <= -X_Inc ) XYZ_desired.x -=X_Inc;

      if(Y_Goal - XYZ_desired.y >= Y_Inc ) XYZ_desired.y +=Y_Inc;
      if(Y_Goal - XYZ_desired.y <= -Y_Inc) XYZ_desired.y -=Y_Inc;   

      if(y_Goal - rpy_desired.z >= yaw_Inc ) rpy_desired.z +=yaw_Inc; //ASDF
      if(y_Goal - rpy_desired.z <= -yaw_Inc) rpy_desired.z -=yaw_Inc;

      if(z_Goal - XYZ_desired.z >= z_Inc ) XYZ_desired.z +=z_Inc;
      if(z_Goal - XYZ_desired.z <= -z_Inc) XYZ_desired.z -=z_Inc;
    }

  }

}


double e_r = 0;
double e_p = 0;
double e_y = 0;

geometry_msgs::Vector3 ang_acl;
geometry_msgs::Vector3 rpy_ddot_d;
Eigen::Vector3d tau_cmd(3);
Eigen::Vector3d rpy_ddot_cmd(3);
void attitude_controller()
{
  
  e_r = rpy_desired.x - imu_rpy.x;
  e_p = rpy_desired.y - imu_rpy.y;
  e_y = rpy_desired.z - imu_rpy.z;

  //ang_acl.x =f (imu_ang_vel.x-prev_ang_vel.x)/delta_t.count();
	//ang_acl.y = (imu_ang_vel.y-prev_ang_vel.y)/delta_t.count();
	//ang_acl.z = (imu_ang_vel.z-prev_ang_vel.z)/delta_t.count();

  e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	
  rpy_ddot_d.x = Par* e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x);//- (double)0.48;
	rpy_ddot_d.y = Par * e_p + Iar * e_p_i + Dar * (-imu_ang_vel.y);//+ (double)0.18; 
	rpy_ddot_d.z = Py * e_y + Dy * (-imu_ang_vel.z);
	
  rpy_ddot_cmd << rpy_ddot_d.x, rpy_ddot_d.y, rpy_ddot_d.z;

  tau_cmd = hat_MoI*rpy_ddot_cmd; // Calculate tau rpy

	tau_rpy_desired.x = tau_cmd(0);
	tau_rpy_desired.y = tau_cmd(1);
	tau_rpy_desired.z = tau_cmd(2);
  
  

}

geometry_msgs::Vector3 lin_vel_desired;
void position_controller()
{
  double e_X=0;
  double e_Y=0;

  // should change SBUS to Ground Station input

  e_X = XYZ_desired.x - position_from_t265.x;
  e_Y = XYZ_desired.y - position_from_t265.y;
  e_X_i += e_X * delta_t.count();
  if (fabs(e_X_i) > position_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * position_integ_limit;
  e_Y_i += e_Y * delta_t.count();
  if (fabs(e_Y_i) > position_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * position_integ_limit;

  XYZ_dot_desired.x = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
  XYZ_dot_desired.y = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
  if(fabs(XYZ_dot_desired.x) > XYZ_dot_limit) XYZ_dot_desired.x = (XYZ_dot_desired.x/fabs(XYZ_dot_desired.x))*XYZ_dot_limit;
  if(fabs(XYZ_dot_desired.y) > XYZ_dot_limit) XYZ_dot_desired.y = (XYZ_dot_desired.y/fabs(XYZ_dot_desired.y))*XYZ_dot_limit;

  lin_vel_desired.x = XYZ_dot_desired.x;
  lin_vel_desired.y = XYZ_dot_desired.y;
}

double X_ddot_d=0;
double Y_ddot_d=0;
double Z_ddot_d=0;
void velocity_controller()
{
  double e_X_dot = 0;
  double e_Y_dot = 0;
  double F_xd=0;
  double F_yd=0;

  e_X_dot = XYZ_dot_desired.x - lin_vel.x;
  e_Y_dot = XYZ_dot_desired.y - lin_vel.y;
  e_X_dot_i += e_X_dot * delta_t.count();
  //limitation
  if (fabs(e_X_dot_i) > velocity_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * velocity_integ_limit;
  e_Y_dot_i += e_Y_dot * delta_t.count();
  if (fabs(e_Y_dot_i) > velocity_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * velocity_integ_limit;
  XYZ_ddot_desired.x = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
  XYZ_ddot_desired.y = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
 
  //limitation
  if(fabs(XYZ_ddot_desired.x) > XYZ_ddot_limit) XYZ_ddot_desired.x = (XYZ_ddot_desired.x/fabs(XYZ_ddot_desired.x))*XYZ_ddot_limit;
  if(fabs(XYZ_ddot_desired.y) > XYZ_ddot_limit) XYZ_ddot_desired.y = (XYZ_ddot_desired.y/fabs(XYZ_ddot_desired.y))*XYZ_ddot_limit;
  X_ddot_d=XYZ_ddot_desired.x;
  Y_ddot_d=XYZ_ddot_desired.y;
/*  F_xyzd.x = mass_system*(W2B_rot(0,0)*XYZ_ddot_desired.x
                   +W2B_rot(0,1)*XYZ_ddot_desired.y
                   +W2B_rot(0,2)*(-XYZ_ddot_desired.z));
  
  F_xyzd.y = mass_system*(W2B_rot(1,0)*(-XYZ_ddot_desired.x)
                   +W2B_rot(1,1)*XYZ_ddot_desired.y
                   +W2B_rot(1,2)*XYZ_ddot_desired.z);*/
  F_xd = mass_system*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d)*sin(imu_rpy.y));
  F_yd = mass_system*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));
  // Force limitation
  if(fabs(F_xd) > F_xd_limit) F_xd = (F_xd/fabs(F_xd))*F_xd_limit;
  if(fabs(F_yd) > F_yd_limit) F_yd = (F_yd/fabs(F_yd))*F_yd_limit;

  F_xd = F_xyzd.x;
  F_yd = F_xyzd.y;

}

void altitude_controller()
{
  
  double e_Z=0;
  double F_zd=0;

  e_Z = XYZ_desired.z - position_from_t265.z;
  e_Z_i += e_Z * delta_t.count();	
  if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

  XYZ_ddot_desired.z = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
  Z_ddot_d = XYZ_ddot_desired.z;
  lin_vel_desired.z = 0; // But this is desired acceleration
  
  // Global to Body ration
	 /* F_xyzd.z = mass_system*(W2B_rot(2,0)*XYZ_ddot_desired.x
                                      +W2B_rot(2,1)*(-XYZ_ddot_desired.y)
                                      +W2B_rot(2,2)*XYZ_ddot_desired.z);*/
  F_zd = mass_system*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
  
       
  //else F_xyzd.z = mass_system*(XYZ_ddot_desired.z);
  if(F_zd > -0.5*mass_system*g) F_zd = -0.5*mass_system*g;
  if(F_zd <= -1.7*mass_system*g) F_zd = -1.7*mass_system*g;
  F_xyzd.z = F_zd;


  //F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
}

Eigen::VectorXd allocation_factor(2);
Eigen::MatrixXd K_M(4,2);
Eigen::MatrixXd invK_M(2,4);
Eigen::VectorXd Dump(4);
void K_matrix() 
{
  double F_xd=F_xyzd.x;
  double F_yd=F_xyzd.y;
  double F_zd=F_xyzd.z;
  double xc = X_c_p1(0);
  double yc = X_c_p1(1);
  double zc = X_c_p1(2);
  Dump << 0,0,0,1;
  K_M << 
  //1x1
  F_yd*zc-F_zd*yc, 
  //1x2
  F_yd*zc-F_zd*(yc+2*l_module),
  //2x1
  -(F_xd*zc-F_zd*xc), 
  //2x2
  -(F_xd*zc-F_zd*xc),
  //3x1
  F_xd*yc-F_yd*xc, 
  //3x2
  F_xd*(yc+2*l_module)-F_yd*xc,
  //4xn
  1,      1;

  invK_M = K_M.completeOrthogonalDecomposition().pseudoInverse();

  allocation_factor = invK_M*Dump;
       
}


void wrench_allocation()
{
  double tau_r=0;
  double tau_p=0;
  double tau_y=0;
  double Fx=0;
  double Fy=0;
  double Fz=0;
  
  //////////// torque distribute ////////////
  tau_r = tau_rpy_desired.x/module_num;
  tau_p = tau_rpy_desired.y/module_num;
  tau_y = tau_rpy_desired.z/module_num;

  tau_rpy_desired.x = tau_r;
  tau_rpy_desired.y = tau_p;
  tau_rpy_desired.z = tau_y;
  
  //////////// force distribute ////////////
  if(!mono_flight){
  F_xyzd_sub1.x = F_xyzd.x*allocation_factor(1);
  F_xyzd_sub1.y = F_xyzd.y*allocation_factor(1);
  F_xyzd_sub1.z = F_xyzd.z*allocation_factor(1);

  Fx = F_xyzd.x*allocation_factor(0);
  Fy = F_xyzd.y*allocation_factor(0);
  Fz = F_xyzd.z*allocation_factor(0);

  F_xyzd.x = 0;
  F_xyzd.y = 0;
  F_xyzd.z = 0;
  }
}

void yaw_torque_distribute()
{
  tau_y_d_non_sat=tau_rpy_desired.z;
  if(fabs(tau_rpy_desired.z) > tau_y_limit)
  {
     tau_rpy_desired.z = tau_rpy_desired.z/fabs(tau_rpy_desired.z)*tau_y_limit;
  }

  if((abs(tau_rpy_desired.z)-tau_y_limit)==0)
  {
	tau_y_th = tau_y_d_non_sat-tau_rpy_desired.z;
	if(fabs(tau_y_th) > tau_y_th_limit) tau_y_th = (tau_y_th/fabs(tau_y_th))*tau_y_th_limit;//2023.08.17 update
  }
}


Eigen::MatrixXd CM_Xc_p2(4,4); //Thrust Allocation
Eigen::MatrixXd invCM_Xc_p2(4,4);
void setCM_Xc_p2()
{ 

  double xc = X_c_p2(0);
  double yc = X_c_p2(1);
  double zc = X_c_p2(2);
  
  double th1 = servo_theta(0);
  double th2 = servo_theta(1);
  double th3 = servo_theta(2);
  double th4 = servo_theta(3);
	CM_Xc_p2 << 
	// 1x1
	(yc+r_arm/r2)*cos(th1)+(-(l_servo-zc)+xi)*sin(th1)/r2,  
	// 1x2
	(yc+r_arm/r2)*cos(th2)+((l_servo-zc)-xi)*sin(th2)/r2,   
	// 1x3
	(yc-r_arm/r2)*cos(th3)+((l_servo-zc)-xi)*sin(th3)/r2,  
	// 1x4
	(yc-r_arm/r2)*cos(th4)+(-(l_servo-zc)+xi)*sin(th4)/r2,
	// 2x1
	-(xc-r_arm/r2)*cos(th1)+((l_servo-zc)+xi)*sin(th1)/r2, 
	// 2x2
	-(xc+r_arm/r2)*cos(th2)+((l_servo-zc)+xi)*sin(th2)/r2, 
	// 2x3
	-(xc+r_arm/r2)*cos(th3)+(-(l_servo-zc)-xi)*sin(th3)/r2, 
	// 2x4
	-(xc-r_arm/r2)*cos(th4)+(-(l_servo-zc)-xi)*sin(th4)/r2,
	// 3x1
	-xi*cos(th1)+(-(xc-yc)/r2)*sin(th1),
	// 3x2
	xi*cos(th2)+((xc+yc)/r2)*sin(th2),
	// 3x3
	-xi*cos(th3)+((xc-yc)/r2)*sin(th3),
	// 3x4
	xi*cos(th4)+(-(xc+yc)/r2)*sin(th4),
	// 4xn
	-cos(th1), 
	-cos(th2), 
	-cos(th3), 
	-cos(th4);
	invCM_Xc_p2 = CM_Xc_p2.inverse();
	
}

double F1=0; // desired motor1 thrust command
double F2=0; // desired motor2 thrust command
double F3=0; // desired motor3 thrust command
double F4=0; // desired motor4 thrust command
Eigen::MatrixXd SA(4,4);
Eigen::MatrixXd invSA(4,4);
void setSA()
{
  SA <<  F1/r2,     F2/r2,     -(F3/r2),     -(F4/r2),
	       F1/r2,    -(F2/r2),   -(F3/r2),       F4/r2,
	      r_arm*F1,  r_arm*F2,   r_arm*F3,   r_arm*F4,
	      r_arm*F1, -(r_arm*F2),   r_arm*F3,  -(r_arm*F4);
  
  invSA=SA.inverse();
}
//////////////////////////////////////

Eigen::VectorXd U(4);
Eigen::VectorXd desired_prop_force(4); // desired motor thrust command
Eigen::VectorXd control_by_theta(4);
Eigen::VectorXd sine_theta_command(4);
std_msgs::Float32MultiArray Force_prop;
double theta1_command=0;
double theta2_command=0;
double theta3_command=0;
double theta4_command=0;
void PWM_signal_Generator()
{

  //////////// Wrench data convert ////////////
  if(!main_agent)
  {
    //sub drone 일 경우 이 위치에서 모든 wrench data를 allocation data로 대체
    F_xyzd.x=wrench_allo_vector(0);
    F_xyzd.y=wrench_allo_vector(1);
    F_xyzd.z=wrench_allo_vector(2);
    tau_rpy_desired.x=wrench_allo_vector(3);
    tau_rpy_desired.y=wrench_allo_vector(4);
    tau_rpy_desired.z=wrench_allo_vector(5);
  }
  ////////////////////////////////////////////

  U << tau_rpy_desired.x, 
       tau_rpy_desired.y, 
       tau_rpy_desired.z, 
       F_xyzd.z;
  desired_prop_force=invCM_Xc_p2*U;

  F1=desired_prop_force(0);
  F2=desired_prop_force(1);
  F3=desired_prop_force(2);
  F4=desired_prop_force(3);
  
  setSA();
  
  control_by_theta << F_xyzd.x, F_xyzd.y, tau_y_th, 0;
  sine_theta_command = invSA*control_by_theta;

  theta1_command = asin(sine_theta_command(0));
  theta2_command = asin(sine_theta_command(1));
  theta3_command = asin(sine_theta_command(2));
  theta4_command = asin(sine_theta_command(3));
  if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
  if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
  if(fabs(theta3_command)>hardware_servo_limit) theta3_command = (theta3_command/fabs(theta3_command))*hardware_servo_limit;
  if(fabs(theta4_command)>hardware_servo_limit) theta4_command = (theta4_command/fabs(theta4_command))*hardware_servo_limit;
  

  pwm_Kill();
  //pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
  
  Force_prop.data.resize(4);
  Force_prop.data[0]=F1;
  Force_prop.data[1]=F2;
  Force_prop.data[2]=F3;
  Force_prop.data[3]=F4;

}

///////// MAIN TO SUB DATA ////////////
std_msgs::String send_data_for_sub;
std::string serial_buffer;
double data1,data2,data3,data4,data5,data6;
bool data7;
std::string data_2_string()
{
  data1=F_xyzd_sub1.x;
  data2=F_xyzd_sub1.y;
  data3=F_xyzd_sub1.z;
  data4=tau_rpy_desired.x;
  data5=tau_rpy_desired.y;
  data6=tau_rpy_desired.z;
  data7=kill_mode;
  

  serial_buffer ="<"+std::to_string(data1)+","
                    +std::to_string(data2)+","
                    +std::to_string(data3)+","
                    +std::to_string(data4)+","
                    +std::to_string(data5)+","
                    +std::to_string(data6)+","
                    +std::to_string(data7)+">";
  return serial_buffer;
}






void PublishData()
{
  //// only main drone case /////
  if(main_agent)
  {
  send_data_for_sub.data = data_2_string(); // send for sub agent data from serial module
  ToSubAgent.publish(send_data_for_sub); // send for sub agent data
  }
  //////////////////////////////
  
  goal_dynamixel_position.publish(servo_msg_create(theta1_command,theta2_command, theta3_command, theta4_command,servo_90rot)); // desired theta

  PWM_generator.publish(PWMs_val); // To ros-pca9685-node
  PWMs.publish(PWMs_cmd);// PWMs_d value
  desired_motor_thrust.publish(Force_prop);// desired force to motor
  
  euler.publish(imu_rpy);//rpy_act value
  desired_angle.publish(rpy_desired);//rpy_d value
  desired_torque.publish(tau_rpy_desired); // torque desired
	
  position.publish(position_from_t265); // actual position
  desired_position.publish(XYZ_desired);//desired position 

  linear_velocity.publish(lin_vel); // actual linear velocity 
  desired_velocity.publish(lin_vel_desired); // desired linear velocity   
  
  desired_force.publish(F_xyzd); // desired force it need only tilt mode 	
  battery_voltage.publish(battery_voltage_msg);
  delta_time.publish(dt);
    
  ///////////////////////Previous Data Shifting///////////////////////////
	
  prev_ang_vel = imu_ang_vel;
  prev_lin_vel = lin_vel;


}

void reset_data()
{

  rpy_desired.z=t265_att.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
  //initial_z=position_from_t265.x;
  e_r_i = 0;
  e_p_i = 0;
  e_Z_i = 0;
  e_X_i=0;
  e_y_i=0;
  
}
