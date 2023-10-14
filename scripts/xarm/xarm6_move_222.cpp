#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/GripperConfig.h>
#include <xarm_msgs/GripperMove.h>
#include <xarm_msgs/MoveVelo.h>
#include <xarm_msgs/SetFloat32.h>
#include <xarm_msgs/RobotMsg.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

float x_diff = 0;
float y_diff = 0;
float z_diff = 0;
float roll_diff = 0;
float pitch_diff = 0;
float yaw_diff = 0;

float error_depth_1,error_pix_x_1,error_pix_y_1;
int center_x = 0;
int center_y = 0;
float img_depth = 0;

float vectorL_x, vectorL_y;
float vectorR_x, vectorR_y;
float xarm_state_x, xarm_state_y, xarm_state_z, xarm_state_x1, xarm_state_y1, xarm_state_z1, xarm_state_start_point_x, xarm_state_start_point_y, xarm_state_start_point_z, xarm_state_end_point_x, xarm_state_end_point_y, xarm_state_end_point_z;
float xarm_state_y11,xarm_state_x11,xarm_state_z11;
float xarm_state_y2,xarm_state_x2,xarm_state_z2;
float xarm_cen_x1, xarm_cen_y1, xarm_cen_x2, xarm_cen_y2, xarm_cen_px, xarm_cen_py;

float r2,r,l1,d,dx,dy;
float kz_s,kz_e,omega,dt,kz,t;
float Px,Py,Px_now,Py_now,Px_diff,Py_diff;
bool camera_get = false;
float  J1,J2,J3,J4,J5,J6;
int STEP_C;
float depth_d,x_d,y_d;

class RosWithClass{
	private:
    ros::NodeHandle nh;
    ros::Publisher sleep_pub,reset_pub;
    ros::Subscriber rgb_depth_sub,vectorL_sub,vectorR_sub,xarm_state_sub,realsense_sub;

    ros::ServiceClient motion_ctrl_client;
    ros::ServiceClient set_mode_client;
  	ros::ServiceClient set_state_client;
    ros::ServiceClient gripper_config_client;
    ros::ServiceClient gripper_move_client;
    ros::ServiceClient velo_move_line_client;
    ros::ServiceClient set_tcp_maxacc_client;
    ros::ServiceClient move_joint_client;
    ros::ServiceClient move_line_tool_client;

    xarm_msgs::SetAxis set_axis_srv;
    xarm_msgs::SetInt16 set_int16_srv;
    xarm_msgs::GripperConfig gripper_config_srv;
    xarm_msgs::GripperMove gripper_move_srv;
    xarm_msgs::MoveVelo move_velo_srv;
    xarm_msgs::SetFloat32 set_float32_srv;
    xarm_msgs::Move move_srv;

    std_msgs::Float32 reset_flag;

   

    
	public:
    int Initial_setting(void);
    int gripper_move_test(int gripper_move);
    int setTcpMaxAcc(float maxacc);
    int veloMoveLine(const std::vector<float>& line_v);
    int move_joint_test(float j1, float j2, float j3, float j4, float j5, float j6, float speed,float acc);
    int move_line_test(float x, float y, float z, float roll, float pitch, float yaw, float speed, float acc);
    int setMode(short mode);
    int setState(short state);
    void Publication(bool reset_count);
    void Callback3(const std_msgs::Float32MultiArray& vectorL);
    void Callback4(const std_msgs::Float32MultiArray& vectorR);
    void Callback5(const xarm_msgs::RobotMsg& xarm_state);
    void Callback6(const std_msgs::Float32MultiArray& realsense);



	RosWithClass(){
    nh.setParam("/xarm/wait_for_finish",true);
    
    reset_pub = nh.advertise<std_msgs::Float32>("resetpub", 1);
    sleep_pub = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);

    vectorL_sub = nh.subscribe("vectorL", 1, &RosWithClass::Callback3, this);
    vectorR_sub = nh.subscribe("vectorR", 1, &RosWithClass::Callback4, this);
    xarm_state_sub = nh.subscribe("/xarm/xarm_states", 1, &RosWithClass::Callback5, this);
    realsense_sub = nh.subscribe("realsense", 1, &RosWithClass::Callback6, this);

    motion_ctrl_client = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
    set_mode_client = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
    set_state_client = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
    gripper_config_client = nh.serviceClient<xarm_msgs::GripperConfig>("/xarm/gripper_config");//speed
    gripper_move_client = nh.serviceClient<xarm_msgs::GripperMove>("/xarm/gripper_move");//open distance
    velo_move_line_client = nh.serviceClient<xarm_msgs::MoveVelo>("/xarm/velo_move_line");
    set_tcp_maxacc_client = nh.serviceClient<xarm_msgs::SetFloat32>("/xarm/set_tcp_maxacc");
    move_joint_client = nh.serviceClient<xarm_msgs::Move>("/xarm/move_joint");
    move_line_tool_client = nh.serviceClient<xarm_msgs::Move>("/xarm/move_line_tool");

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
  gripper_config_srv.request.pulse_vel = 3000;
  if(gripper_config_client.call(gripper_config_srv)){
    ROS_INFO("%s\n", gripper_config_srv.response.message.c_str());
  }
  else{
    ROS_ERROR("Failed to call service gripper_config");
    return 1;
  } 
/**************************************************/
}

int RosWithClass::setMode(short mode){
	set_int16_srv.request.data = mode;
  set_mode_client.call (set_int16_srv);
  }

int RosWithClass::setState(short state){
	set_int16_srv.request.data = state;
  set_state_client.call(set_int16_srv);
  }

int RosWithClass::gripper_move_test(int gripper_move){
  /************************グリッパー動作***********************/
  gripper_move_srv.request.pulse_pos = gripper_move;
  if(gripper_move_client.call(gripper_move_srv)){
    ROS_INFO("%s\n",  gripper_move_srv.response.message.c_str());
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
  }


int RosWithClass::veloMoveLine(const std::vector<float>& line_v){
  move_velo_srv.request.velocities = line_v;
  move_velo_srv.request.coord = 1;
  velo_move_line_client.call(move_velo_srv);
  }


int RosWithClass::move_joint_test(float j1, float j2, float j3, float j4, float j5, float j6, float speed,float acc){
  move_srv.request.mvvelo = speed;
  move_srv.request.mvacc = acc;
  move_srv.request.mvtime = 0;

  move_srv.request.pose ={j1, j2, j3, j4, j5, j6};

  move_joint_client.call(move_srv);
    
  return 0;
}

int RosWithClass::move_line_test( float x, float y, float z, float roll, float pitch, float yaw, float speed, float acc){
  //セイフティー
  //if(z > 106) z = 106;
  //if(speed > 200) speed = 200;
  //if(acc > 1000) acc = 1000;

  move_srv.request.mvvelo = speed;
  move_srv.request.mvacc = acc;
  move_srv.request.mvtime = 0;

  float x_now = x;
  float y_now = y;
  float z_now = z;
  float roll_now = roll;
  float pitch_now = pitch;
  float yaw_now = yaw;

  x_diff = x;
  y_diff = y;
  z_diff = z;
  roll_diff = roll;
  pitch_diff = pitch;
  yaw_diff = yaw;


  move_srv.request.pose ={x_now, y_now, z_now, roll_now, pitch_now, yaw_now};
  move_line_tool_client.call(move_srv);
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

void RosWithClass::Callback3( const std_msgs::Float32MultiArray& vectorL ){
  vectorL_x = vectorL.data[0];
  vectorL_y = vectorL.data[1];
}

void RosWithClass::Callback4( const std_msgs::Float32MultiArray& vectorR ){
  vectorR_x = vectorR.data[0];
  vectorR_y = vectorR.data[1];
}

void RosWithClass::Callback5( const xarm_msgs::RobotMsg& xarm_state ){
  xarm_state_x =  xarm_state.pose[0];
  xarm_state_y =  xarm_state.pose[1];
  xarm_state_z =  xarm_state.pose[2];

  J1 = xarm_state.angle[0];
  J2 = xarm_state.angle[1];
  J3 = xarm_state.angle[2];
  J4 = xarm_state.angle[3];
  J5 = xarm_state.angle[4];
  J6 = xarm_state.angle[5];

}

void RosWithClass::Callback6( const std_msgs::Float32MultiArray& realsense ){
  depth_d =  realsense.data[0];
  x_d =  realsense.data[1];
  y_d =  realsense.data[2];
}

int main(int argc, char **argv){
  ros::init(argc, argv, "xarm6_move_222");
  ros::NodeHandle nh;
  RosWithClass ros_with_class; 
  nh.setParam("/xarm/wait_for_finish",true);

  ros_with_class.Initial_setting();
  ros_with_class.setMode(0);
  ros_with_class.setState(0);
  ros::Rate loop_rate(10);
  //全関節角度0
  ros_with_class.move_joint_test(0, 0, 0, 0, 0, 0, 0.35, 5);
  ros::spinOnce();
  ros::Duration(2).sleep();
  //手先を300mm前に
  ros_with_class.move_line_test(300, 0, 0,  0, 0, 0, 100, 1000);
  ros::spinOnce();
  ros::Duration(2).sleep();
  
  //5関節目　角度-90
  ros_with_class.move_joint_test(0, 0, 0, 0, -1.5707, 0, 0.35, 5);
  ros::spinOnce();
  ros::Duration(2).sleep();
  
  //グリッパ開く
  ros_with_class.gripper_move_test(800);
  ros::spinOnce();
  ros::Duration(2).sleep();
  //グリッパ閉じる
  ros_with_class.gripper_move_test(100);
  ros::spinOnce();
  ros::Duration(5).sleep();
  //速度制御　前方に50mm/s
  nh.setParam("/xarm/wait_for_finish",false);
  ros_with_class.setMode(5);
  ros_with_class.setState(0);
  ros_with_class.setTcpMaxAcc(50000);

  std::vector<float> line_v = { 0, 0, 50, 0, 0, 0};
  ros_with_class.veloMoveLine(line_v);
  ros::spinOnce();
  ros::Duration(5).sleep();

  line_v = { 0, 0, 0, 0, 0, 0};
  ros_with_class.veloMoveLine(line_v);
  ros::spinOnce();
  

}