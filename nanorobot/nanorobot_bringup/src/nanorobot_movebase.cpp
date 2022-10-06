#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nanorobot_msgs/Velocities.h>
#include <nanorobot_msgs/Quat.h>
#include <nanorobot_msgs/Imu.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>

double g_vel_x = 0.0;
double g_vel_y = 0.0;

double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;

double quatyaw = 0.0;

ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);

sensor_msgs::Imu imu;

void velCallback( const nanorobot_msgs::Velocities& vel) {
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();

  g_vel_x = vel.linear_x;
  g_vel_y = vel.linear_y;

  g_vel_dt = (current_time - g_last_vel_time).toSec();
  g_last_vel_time = current_time;
}
// 
void IMUCallback( const nanorobot_msgs::Imu& raw_imu){
  quatyaw = tf::getYaw(raw_imu.orientation);
  imu.orientation = raw_imu.orientation;
  imu.angular_velocity = raw_imu.angular_velocity;
  imu.linear_acceleration = raw_imu.linear_acceleration;
}
// 
// void QuatCallback(const nanorobot_msgs::Quat& quat){
//   quatyaw = tf::getYaw(quat.orientation);
// }

int main(int argc, char** argv){
  double angular_scale, linear_scale;
  int data_update_freq;
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_vel", 50, velCallback);
  ros::Subscriber imu_sub = n.subscribe("raw_imu", 50, IMUCallback);
  // ros::Subscriber quat_sub = n.subscribe("raw_quat", 50, QuatCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 50);
  
  tf::TransformBroadcaster odom_broadcaster;

  nh_private_.getParam("angular_scale", angular_scale);
  nh_private_.getParam("linear_scale", linear_scale);
  nh_private_.getParam("data_update_freq", data_update_freq);
  double rate = data_update_freq;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;
  double previous_theta = 0.0;
  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();

    //linear velocity is the linear velocity published from the Teensy board in x axis
    double linear_velocity_x = g_vel_x;

    //linear velocity is the linear velocity published from the Teensy board in y axis
    double linear_velocity_y = g_vel_y;

    //angular velocity is the rotation in Z from imu_filter_madgwick's output
    double angular_velocity = g_imu_z;
    theta = quatyaw;
    previous_theta = 0;
    //calculate angular displacement  θ = ω * t
    double delta_theta = angular_velocity * g_imu_dt * angular_scale; //radians
    double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * g_vel_dt * linear_scale; //m
    double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * g_vel_dt * linear_scale; //m

    //calculate current position of the robot
    x_pos += delta_x;
    y_pos += delta_y;
    // theta += delta_theta;
    
    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    // tf::yaw

    // geometry_msgs::Quaternion odom_quat = imu
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(odom_trans);

    // imu.orientation_covariance
    imu.header.frame_id = "imu_raw";
    imu.header.stamp = current_time;
    // imu_pub.publish(imu);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x;
    odom.twist.twist.linear.y = linear_velocity_y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from IMU
    odom.twist.twist.angular.z = imu.angular_velocity.z;

    odom.pose.covariance = {
        1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3, 1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9};
    odom.twist.covariance = {
        1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3, 1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9};

    odom_pub.publish(odom);

    g_last_loop_time = current_time;
    r.sleep();
  }
}
