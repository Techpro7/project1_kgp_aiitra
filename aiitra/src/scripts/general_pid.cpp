#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>

using namespace std;

/*defining global parameters*/
double g=9.81,m=3.50,Izz=0.07;
double lin_Kp=10,lin_Kd=2;
double ang_Kp=10,ang_Kd=2;
double t_res=0.001,k=0.1;


struct Quaternion {
    double w, x, y, z;
} Q;

struct EulerAngles {
    double roll, pitch, yaw;
} E;

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

struct robot
{
  string name;
  int num;
  float pos[3];
  float orientation[4];
  float linear_vel[3];
  float angular_vel[3];

  float goal[2];
} roomba;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	int i;
	for (i = 0;; ++i)
	{
		if( (msg->name[i]).compare(roomba.name)  )
			break;
	}

	roomba.pos[0] = msg->pose[i].position.x;
	roomba.pos[1] = msg->pose[i].position.y;
	roomba.pos[2] = msg->pose[i].position.z;

	roomba.orientation[0] = msg->pose[i].orientation.x;
	roomba.orientation[1] = msg->pose[i].orientation.y;
	roomba.orientation[2] = msg->pose[i].orientation.z;
	roomba.orientation[3] = msg->pose[i].orientation.w;

	roomba.linear_vel[0] = msg->twist[i].linear.x;
	roomba.linear_vel[1] = msg->twist[i].linear.y;
	roomba.linear_vel[2] = msg->twist[i].linear.z;

	roomba.angular_vel[0] = msg->twist[i].angular.x;
	roomba.angular_vel[1] = msg->twist[i].angular.y;
	roomba.angular_vel[2] = msg->twist[i].angular.z;

  	Q.x = roomba.orientation[0];
    Q.y = roomba.orientation[1];
    Q.z = roomba.orientation[2];
    Q.w = roomba.orientation[3];
}

void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
  roomba.goal[0] = msg->poses[0].pose.position.x;
  roomba.goal[1] = msg->poses[0].pose.position.y;
}


Eigen::Vector3d PID(){

	double err_x = roomba.goal[0] - roomba.pos[0];
	double err_y = roomba.goal[1] - roomba.pos[1];

	double Ux = lin_Kp*err_x + lin_Kd*(0 - roomba.linear_vel[0]);
	double Uy = lin_Kp*err_y + lin_Kd*(0 - roomba.linear_vel[1]);

	double des_ang = atan2(Uy,Ux);

	E = ToEulerAngles(Q);

	double err_ang = des_ang - E.yaw;

	double Uz = ang_Kp*err_ang + ang_Kd*(0 - roomba.linear_vel[2]);

	Eigen::Vector3d U(Ux, Uy, Uz);

	return U;

}


int main(int argc, char **argv){

	ros::init(argc, argv, "pid_node");
	ros::NodeHandle n("~");

	//n.getParam("botName", roomba.name);
	roomba.name = "roomba1";
	roomba.num = roomba.name[6];

	string traj_topic = "/"+roomba.name+"/trajectory";

	ros::Subscriber odom_sub = n.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
	//ros::Subscriber traj_sub = n.subscribe(traj_topic, 1000, trajectoryCallback);

	ros::Publisher left_pub = n.advertise<std_msgs::Float64>("/"+roomba.name+"/left_joint_velocity_controller/command", 1);
	ros::Publisher right_pub = n.advertise<std_msgs::Float64>("/"+roomba.name+"/right_joint_velocity_controller/command", 1);
	ros::Publisher back_pub = n.advertise<std_msgs::Float64>("/"+roomba.name+"/back_joint_velocity_controller/command", 1);

	cout<<"Enter x : ";
	cin>>roomba.goal[0];
	cout<<"Enter y : ";
	cin>>roomba.goal[1];

	ros::Rate r(50);

	while(n.ok()){

		ros::spinOnce();

		Eigen::Vector3d U = PID();

		Eigen::Matrix3d Inv;

		Inv<< -0.86602540378,		0.5,		0.34,
			0,						-1,			0.34,
			0.86602540378,			0.5,		0.34;

		Eigen::Vector3d V_(U[0]*cos(E.yaw)+U[1]*sin(E.yaw), U[1]*cos(E.yaw)-U[0]*sin(E.yaw), U[2]);

		Eigen::Vector3d vel = Inv*V_;

		std_msgs::Float64 msg_left, msg_right, msg_back;

		msg_left.data = vel[0];
		msg_right.data = vel[1];
		msg_back.data = vel[2];

		left_pub.publish(msg_left);
		right_pub.publish(msg_right);
		back_pub.publish(msg_back);

		r.sleep();
	}
}