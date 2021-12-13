#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

/*
name: [ground_plane, open_base]
pose: 
  - 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - 
    position: 
      x: 0.00189637636614
      y: -0.00569978751731
      z: -2.82333844277e-05
    orientation: 
      x: 2.91289134543e-06
      y: -6.18839296807e-06
      z: -0.301024726322
      w: 0.953616334852
twist: 
  - 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  - 
    linear: 
      x: -0.000837178776827
      y: 0.00340879983085
      z: 0.00181465803117
    angular: 
      x: -0.00309812648786
      y: -0.00475377285284
      z: -0.00279537168965
---
~ $ rostopic info /gazebo/model_states 
Type: gazebo_msgs/ModelStates

Publishers: 
 * /gazebo (http://devraj-LEGION-Y530:43003/)

~ $ rosmsg info gazebo_msgs/ModelState
string model_name
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
string reference_frame
*/

struct robot_state
{
  float pos[3];
  float orientation[4];
  float linear_vel[3];
  float angular_vel[3];
} open_base;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  open_base.pos[0] = msg->pose[1].position.x;
  open_base.pos[1] = msg->pose[1].position.y;
  open_base.pos[2] = msg->pose[1].position.z;

  open_base.orientation[0] = msg->pose[1].orientation.x;
  open_base.orientation[1] = msg->pose[1].orientation.y;
  open_base.orientation[2] = msg->pose[1].orientation.z;
  open_base.orientation[3] = msg->pose[1].orientation.w;

  open_base.linear_vel[0] = msg->twist[1].linear.x;
  open_base.linear_vel[1] = msg->twist[1].linear.y;
  open_base.linear_vel[2] = msg->twist[1].linear.z;

  open_base.angular_vel[0] = msg->twist[1].angular.x;
  open_base.angular_vel[1] = msg->twist[1].angular.y;
  open_base.angular_vel[2] = msg->twist[1].angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50);
  while(n.ok()){

    ros::spinOnce();

    current_time = ros::Time::now();

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    //since all odometry is 6DOF we'll need a quaternion created from yaw

    /*
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation.x = 0;
    odom_trans.transform.rotation.y = 0;
    odom_trans.transform.rotation.z = 0;
    odom_trans.transform.rotation.w = 1;
    */
    
    odom_trans.transform.translation.x = open_base.pos[0];
    odom_trans.transform.translation.y = open_base.pos[1];
    odom_trans.transform.translation.z = open_base.pos[2];
    odom_trans.transform.rotation.x = open_base.orientation[0];
    odom_trans.transform.rotation.y = open_base.orientation[1];
    odom_trans.transform.rotation.z = open_base.orientation[2];
    odom_trans.transform.rotation.w = open_base.orientation[3];
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = open_base.pos[0];
    odom.pose.pose.position.y = open_base.pos[1];
    odom.pose.pose.position.z = open_base.pos[2];
    odom.pose.pose.orientation.x = open_base.orientation[0];
    odom.pose.pose.orientation.y = open_base.orientation[1];
    odom.pose.pose.orientation.z = open_base.orientation[2];
    odom.pose.pose.orientation.w = open_base.orientation[3];

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = open_base.linear_vel[0];
    odom.twist.twist.linear.y = open_base.linear_vel[1];
    odom.twist.twist.angular.z = open_base.angular_vel[2];

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();

  }
}
