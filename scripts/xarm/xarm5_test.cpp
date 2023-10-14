#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/GripperConfig.h>
#include <xarm_msgs/GripperConfigRequest.h>
#include <xarm_msgs/GripperConfigResponse.h>
#include <xarm_msgs/GripperMove.h>
#include <xarm_msgs/MoveVelo.h>
#include <xarm_msgs/SetFloat32.h>
#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/GripperState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <math.h>


float x_diff = 0;
float y_diff = 0;
float z_diff = 0;
float roll_diff = 0;
float pitch_diff = 0;
float yaw_diff = 0;

float xarm_state_x, xarm_state_y, xarm_state_z, xarm_state_x1, xarm_state_y1, xarm_state_z1, xarm_state_start_point_x, xarm_state_start_point_y, xarm_state_start_point_z, xarm_state_end_point_x, xarm_state_end_point_y, xarm_state_end_point_z;
float xarm_state_y11,xarm_state_x11,xarm_state_z11;
float xarm_state_y2,xarm_state_x2,xarm_state_z2;
float xarm_cen_x1, xarm_cen_y1, xarm_cen_x2, xarm_cen_y2, xarm_cen_px, xarm_cen_py;
float gripper_state_data, power_data;

float r2,r,l1,d,dx,dy;
float kz_s,kz_e,omega,dt,kz,t;
float Px,Py,Px_now,Py_now,Px_diff,Py_diff;
float J1,J2,J3,J4,J5;


class RosWithClass{
	private:
    ros::NodeHandle nh;
    ros::Publisher sleep_pub,reset_pub;
    ros::Subscriber xarm_state_sub, power_sub;


    ros::ServiceClient motion_ctrl_client;
    ros::ServiceClient set_mode_client;
  	ros::ServiceClient set_state_client;
    ros::ServiceClient gripper_config_client;
    ros::ServiceClient gripper_move_client;
    ros::ServiceClient velo_move_line_client;
    ros::ServiceClient set_tcp_maxacc_client;
    ros::ServiceClient move_joint_client;
    ros::ServiceClient move_line_tool_client;
    ros::ServiceClient gripper_state_client;

    xarm_msgs::SetAxis set_axis_srv;
    xarm_msgs::SetInt16 set_int16_srv;
    xarm_msgs::GripperConfig gripper_config_srv;
    xarm_msgs::GripperMove gripper_move_srv;
    xarm_msgs::MoveVelo move_velo_srv;
    xarm_msgs::SetFloat32 set_float32_srv;
    xarm_msgs::Move move_srv;
    xarm_msgs::GripperState gripperstate_srv;

    std_msgs::Float32 reset_flag;

   

    
	public:
    int Initial_setting(void);
    int gripper_move_test(int gripper_move);
    int setTcpMaxAcc(float maxacc);
    int veloMoveLine(const std::vector<float>& line_v);
    int move_joint_test(float j1, float j2, float j3, float j4, float j5, float speed,float acc);
    int move_line_test(float x, float y, float z, float roll, float pitch, float yaw, float speed, float acc);
    int setMode(short mode);
    int setState(short state);
    void Publication(bool reset_count);
    void Callback_xarm(const xarm_msgs::RobotMsg& xarm_state);

    int Gripper_state(void);
    void Callback_power(const std_msgs::Float32& power);



	RosWithClass(){
    nh.setParam("/xarm/wait_for_finish",true);
    
    reset_pub = nh.advertise<std_msgs::Float32>("resetpub", 1);
    sleep_pub = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);

    xarm_state_sub = nh.subscribe("/xarm/xarm_states", 1, &RosWithClass::Callback_xarm, this);
    power_sub = nh.subscribe("power", 1, &RosWithClass::Callback_power, this);

    motion_ctrl_client = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
    set_mode_client = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
    set_state_client = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
    gripper_config_client = nh.serviceClient<xarm_msgs::GripperConfig>("/xarm/gripper_config");//speed
    gripper_move_client = nh.serviceClient<xarm_msgs::GripperMove>("/xarm/gripper_move");//open distance
    velo_move_line_client = nh.serviceClient<xarm_msgs::MoveVelo>("/xarm/velo_move_line");
    set_tcp_maxacc_client = nh.serviceClient<xarm_msgs::SetFloat32>("/xarm/set_tcp_maxacc");
    move_joint_client = nh.serviceClient<xarm_msgs::Move>("/xarm/move_joint");
    move_line_tool_client = nh.serviceClient<xarm_msgs::Move>("/xarm/move_line_tool");
    gripper_state_client = nh.serviceClient<xarm_msgs::GripperState>("/xarm/gripper_state");
  }
};



int RosWithClass::Initial_setting(void){
/************************初期設定***********************/
  set_axis_srv.request.id = 8;
  set_axis_srv.request.data = 1;
  if(motion_ctrl_client.call(set_axis_srv)){
    ROS_INFO("%s\n", set_axis_srv.response.message.c_str());
  }
  else{
    ROS_ERROR("Failed to call service motion_ctrl");
    return 1;
  }
  //mode 0
  set_int16_srv.request.data = 0;
  if(set_mode_client.call(set_int16_srv)){
    ROS_INFO("%s\n", set_int16_srv.response.message.c_str());
  }
  else{
    ROS_ERROR("Failed to call service set_mode");
     return 1;
  }  
  //state 0
  set_int16_srv.request.data = 0;
  if(set_state_client.call(set_int16_srv)){
    ROS_INFO("%s\n", set_int16_srv.response.message.c_str());
  }
  else{
      ROS_ERROR("Failed to call service set_state");
      return 1;
  }
  //グリッパーの速度設定
  gripper_config_srv.request.pulse_vel = 2000;
  if(gripper_config_client.call(gripper_config_srv)){
    ROS_INFO("%s\n", gripper_config_srv.response.message.c_str());
  }
  else{
    ROS_ERROR("Failed to call service gripper_config");
    return 1;
  }
  return 0;
/**************************************************/
}

int RosWithClass::setMode(short mode){
	set_int16_srv.request.data = mode;
  set_mode_client.call (set_int16_srv);
  return 0;
  }

int RosWithClass::setState(short state){
	set_int16_srv.request.data = state;
  set_state_client.call(set_int16_srv);
  return 0;
  }

int RosWithClass::gripper_move_test(int gripper_move){
  /************************グリッパー動作***********************/
  gripper_move_srv.request.pulse_pos = gripper_move;
  if(gripper_move_client.call(gripper_move_srv)){
    ROS_INFO("%s",  gripper_move_srv.response.message.c_str());
  }
  else{
    ROS_ERROR("Failed to call service gripper_move");
    return 1;
  }
  return 0;
}

int RosWithClass::setTcpMaxAcc(float maxacc){
  set_float32_srv.request.data = maxacc;
  set_tcp_maxacc_client.call(set_float32_srv);
  return 0;
  }


int RosWithClass::veloMoveLine(const std::vector<float>& line_v){
  move_velo_srv.request.velocities = line_v;
  move_velo_srv.request.coord = 1;
  velo_move_line_client.call(move_velo_srv);
  return 0;
  }

int RosWithClass::move_joint_test(float j1, float j2, float j3, float j4, float j5, float speed,float acc){
  //セイフティー
  if(speed > 0.35) speed = 0.35;
  if(acc > 7) acc = 7;

  move_srv.request.mvvelo = speed;
  move_srv.request.mvacc = acc;
  move_srv.request.mvtime = 0;
  move_srv.request.mvradii = 20;
  move_srv.request.pose = {j1, j2, j3, j4, j5};
  move_joint_client.call(move_srv);
  if (move_joint_client.call(move_srv)) {
        ROS_INFO("%s\n", move_srv.response.message.c_str());
    } else {
        ROS_ERROR("Failed to call Move service.");
        return 1;
  }
  return 0;
}



int RosWithClass::move_line_test( float x, float y, float z, float roll, float pitch, float yaw, float speed, float acc){
  //セイフティー
  if(z > 106) z = 106;
  if(speed > 200) speed = 200;
  if(acc > 1000) acc = 1000;

  move_srv.request.mvvelo = speed;
  move_srv.request.mvacc = acc;
  move_srv.request.mvtime = 0;
  move_srv.request.mvradii = 20;

  move_srv.request.pose = {x, y, z, roll, pitch, yaw};
  move_line_tool_client.call(move_srv);

  if (move_line_tool_client.call(move_srv)) {
        ROS_INFO("%s\n", move_srv.response.message.c_str());
    } else {
        ROS_ERROR("Failed to call Move service.");
        return 1;
  }

  return 0;
}

void RosWithClass::Publication(bool reset_count)
{
  if(reset_count == true){
    reset_flag.data = 1.0f;
    reset_pub.publish(reset_flag);
  }
   if(reset_count == false){
    reset_flag.data = -1.0f;
    reset_pub.publish(reset_flag);
  }
}


void RosWithClass::Callback_xarm( const xarm_msgs::RobotMsg& xarm_state ){
  xarm_state_x =  xarm_state.pose[0];
  xarm_state_y =  xarm_state.pose[1];
  xarm_state_z =  xarm_state.pose[2];

  J1 = xarm_state.angle[0];
  J2 = xarm_state.angle[1];
  J3 = xarm_state.angle[2];
  J4 = xarm_state.angle[3];
  J5 = xarm_state.angle[4];
}

int RosWithClass::Gripper_state(void){
  
  if (gripper_state_client.call(gripperstate_srv)) {
      gripper_state_data = gripperstate_srv.response.curr_pos;
      ROS_INFO("Gripper State: %f", gripper_state_data);
    } else {
      ROS_ERROR("Failed to call GetGripperState service");
      return 1;
  }
  return 0;

}

void RosWithClass::Callback_power( const std_msgs::Float32& power ){
  power_data = power.data;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "xarm5_test");
  ros::NodeHandle nh;
  RosWithClass ros_with_class; 
  nh.setParam("/xarm/wait_for_finish",true);

  ros_with_class.Initial_setting();
  ros_with_class.setMode(0);
  ros_with_class.setState(0);
  ros_with_class.Gripper_state();
  ros::Rate loop_rate(10);


  /*
  ros_with_class.move_line_test(295, 0, 0, 0, 0, 0, 10, 100);
  ros::spinOnce();
  ros::Duration(2).sleep();
  */
  /*
  ros_with_class.move_line_test(228.06085205078125, 4.841962622542011*exp(-15), 284.92474365234375, -3.1415927410125732, -1.5700000524520874, -7.689852986152157*exp(-14), 10, 100);
  ros::spinOnce();
  ros::Duration(2).sleep();
  */
 
 

  ros_with_class.move_joint_test(0, 0, 0, -1.57, 0, 0.3, 3);
  ros::spinOnce();
  ros::Duration(2).sleep();

  ros_with_class.move_joint_test(0, 0, -0.535, 0.558, 0, 0.3, 3);
  ros::spinOnce();
  ros::Duration(2).sleep();
  
  //グリッパ
  ros_with_class.gripper_move_test(400);
  ros::spinOnce();
  ros::Duration(3).sleep();


  float gripper_state_limit = 50;
  float power_goal = 0.45;
  float gain = 8;

  while(ros::ok()){

    ros_with_class.Gripper_state();
    ROS_INFO("power = %f\n",  power_data);

    if(power_data < power_goal){
      
      if(gripper_state_data - gain > gripper_state_limit){
        ros_with_class.gripper_move_test(gripper_state_data - gain );
        //ros_with_class.gripper_move_test(gripper_state_data + (power_data - power_goal)*gain );
        ros::spinOnce();
      }
      
    }
    else{
      ros_with_class.move_joint_test(0, 0, 0, -1.57, 0, 0.3, 3);
      ROS_INFO("Finished\n\n");
      break;
    }
    ros::Duration(0.0001).sleep();
    

  }

  

}
