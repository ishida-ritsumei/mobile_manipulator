#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


double vx =  0.0;
double vy =  0.0;
double vth = 0.0;
double odom_kv = 1.0;
double odom_kth = 1.0;
double x_sum = 0.0;
double y_sum = 0.0;
double vth_sum = 0.0;

int receive_flag = 0;

void roverOdomCallback(const geometry_msgs::Twist::ConstPtr& rover_odom){
  vx = odom_kv * rover_odom->linear.x;
  vy = odom_kv * rover_odom->linear.y;
  vth = odom_kth * rover_odom->angular.z;
  receive_flag = 1;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv,"vstone");
  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("rover_twist", 1);
  ros::Subscriber odom_sub = n.subscribe("/rover_odo", 100, roverOdomCallback);
  geometry_msgs::Twist twist;
  ros::Rate loop_rate(10);
  ros::Time current_time, last_time;

  current_time = ros::Time::now();
  last_time = ros::Time::now();
  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(y_sum>0.5){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    
    twist.angular.z =0;// rad/s
    twist.linear.x = 0.0;// m/s
    twist.linear.y = 0.2;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(y_sum<0){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    twist.angular.z =0;// rad/s
    twist.linear.x = 0;// m/s
    twist.linear.y = -0.2;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(y_sum>0.3){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    
    twist.angular.z =0;// rad/s
    twist.linear.x = 0.2;// m/s
    twist.linear.y = 0.2;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(y_sum<0){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    twist.angular.z =0;// rad/s
    twist.linear.x = -0.2;// m/s
    twist.linear.y = -0.2;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(vth_sum>1.57){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    twist.angular.z =0.4;// rad/s
    twist.linear.x = 0;// m/s
    twist.linear.y = 0;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(vth_sum<-0.785){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    twist.angular.z =-0.4;// rad/s
    twist.linear.x = 0;// m/s
    twist.linear.y = 0;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::Duration(2).sleep();  
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(vth_sum) - vy * sin(vth_sum)) * dt;
    double delta_y = (vx * sin(vth_sum) + vy * cos(vth_sum)) * dt;
    double delta_th = vth * dt;

    x_sum +=delta_x;
    y_sum +=delta_y;
    vth_sum +=delta_th;
    ROS_INFO("x=%lf, y=%lf, z=%lf\n",x_sum,y_sum,vth_sum);

    if(vth_sum>0){
      twist.angular.z =0;// rad/s
      twist.linear.x = 0;// m/s
      twist.linear.y = 0;// m/s
      move_pub.publish(twist);
      break;
    }
    twist.angular.z =0.3;// rad/s
    twist.linear.x = 0;// m/s
    twist.linear.y = 0;// m/s
    move_pub.publish(twist);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }


  


}

