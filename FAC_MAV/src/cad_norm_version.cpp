
#include "cad_uav_controller.hpp"

//직접 계산에 사용하는 변수는 이름을 축약해서
//퍼를리쉬 하기 위해 선언한 변수는 길게

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cad_uav");
  ros::NodeHandle nh;

  
  //integratior(PID) limitation
                integ_limit=nh.param<double>("attitude_integ_limit",10);
                integ_yaw_limit=nh.param<double>("attitude_y_integ_limit",10);
                z_integ_limit=nh.param<double>("altitude_integ_limit",100);
                position_integ_limit=nh.param<double>("position_integ_limit",10);

                CoM_hat.x = nh.param<double>("x_center_of_mass",1.0);
                CoM_hat.y = nh.param<double>("y_center_of_mass",1.0);
                CoM_hat.z = nh.param<double>("z_center_of_mass",1.0);

                        //Roll, Pitch PID gains

                        tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
                        tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
                        tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);

                        tilt_Pap=nh.param<double>("tilt_attitude_p_P_gain",3.5);
                        tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",3.5);
                        tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",3.5);

                        //Yaw PID gains
                        tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
                        tilt_Iy=nh.param<double>("tilt_attitude_y_I_gain",5.0);
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

  //initialize ros node//
  //initSubscriber();

    /////////////////////////////////////////////////SUBSCFRIBER START//////////////////////////////////////////////////////
    dynamixel_state = nh.subscribe("joint_states",100,jointstate_Callback, ros::TransportHints().tcpNoDelay());
    att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    rc_in = nh.subscribe("/sbus",100,sbus_Callback,ros::TransportHints().tcpNoDelay());
    battery_checker = nh.subscribe("/battery",100,battery_Callback,ros::TransportHints().tcpNoDelay());
    t265_position=nh.subscribe("/t265_pos",100,t265_position_Callback,ros::TransportHints().tcpNoDelay());
    t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265_Odom_Callback,ros::TransportHints().tcpNoDelay());

    /////////////////////////////////////////////////PUBLISHER START//////////////////////////////////////////////////////
    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); 
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("command",1);  // publish to pca9685
    desired_motor_thrust = nh.advertise<std_msgs::Float32MultiArray>("Forces",100);

    goal_dynamixel_position  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2

    euler = nh.advertise<geometry_msgs::Vector3>("angle",1); 
    desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);


    desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);

    linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
    desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",100);

    angular_velocity = nh.advertise<geometry_msgs::Vector3>("ang_vel",100);

    desired_position = nh.advertise<geometry_msgs::Vector3>("position_d",100);
    position = nh.advertise<geometry_msgs::Vector3>("position",100);

    desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);

    battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
    delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);

    ToSubAgent = nh.advertise<std_msgs::String>("ToSubData",1);

 
  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    Clock();
    if(!kill_mode)
    {
    shape_detector();
    UpdateParameter(); 
    //setMoI,pid_Gain_Setting, etc.

    //if(){
    Command_Generator();
    attitude_controller();
    position_controller();
    altitude_controller();

    Accelerometer_LPF();
    velocity_controller();
    K_matrix();
    wrench_allocation();
    yaw_torque_distribute();

    PWM_signal_Generator(); 
    //contain :: setCM,setSA, etc
    //}
    }
    else
    {
      
      reset_data();
      pwm_Kill();
      
    }

    PublishData();
    ros::spinOnce();
    loop_rate.sleep();
  }
  





  return 0;
}



