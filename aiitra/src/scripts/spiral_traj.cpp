#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>

/*
~/Projects/grid_challenge/src/project1_kgp_aiitra $ rosmsg info nav_msgs/Path
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
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

void trajectoryStatesCallback(const nav_msgs::Path::ConstPtr& msg)
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

int main(int argc, char **argv){

	ros::init(argc, argv, "spiral_traj");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
	ros::Publisher traj_pub = n.advertise<nav_msgs::Path>("trajectory", 50);

	ros::Rate r(50);

	while(n.ok()){

		ros::spinOnce();

		nav_msgs::Path traj;



	}


}