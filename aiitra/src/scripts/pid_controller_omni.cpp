#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include"stdafx.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>


using namespace std;

/*defining global parameters*/
global double g=9.81,m=3.50,Izz=0.07;
global double Kp=1,Kd=1;
global double t_res=0.001,k=0.1;

/*defining the controller function*/
vector<double> error_pos;
vector<double> error_vel;
vector<double> desired_state_pos;
vector<double> current_state_pos;
vector<double> desired_state_vel;
vector<double> current_state_vel;
vector<double> desired_state_acc;
vector<double> current_state_acc;

void TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg){

	Eigen::Vector3d des_posW, des_velW, des_accW, des_jerkW, des_snapW, des_eulW, des_omgW;
	Eigen::Quaternion<double> des_quatW;

	mav_msgs::EigenTrajectoryPoint traj_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &traj_reference);

	des_posW = traj_reference.position_W;
	des_velW = traj_reference.velocity_W;
	des_accW = traj_reference.acceleration_W;
	des_jerkW = traj_reference.jerk_W;
	des_snapW = traj_reference.snap_W;

	des_quatW = traj_reference.orientation_W_B;
	mav_msgs::getEulerAnglesFromQuaternion(des_quatW, &des_eulW);
	des_omgW = traj_reference.angular_velocity_W;

	desState.position = des_posW;
	desState.velocity = des_velW;
	desState.acceleration = des_accW;
	desState.euler_angle = des_eulW;
	desState.angular_velocity = des_omgW;
}




for(int i=0;i<3;i++)
{
 error_pos[i]=current_state_pos[i]-desired_state_pos[i];
 error_vel[i]=current_state_vel[i]-desired_state_vel[i];
 
 
}


vector<double> controller(vector<double> error_pos;vector<double> error_vel;vector<double> desired_state_acc)
{
/**defining the conrol law*/
vector<double> current_state_acc;
double f[3];//forces on the wheel of the bot
vector<double> omega_wheel;
for(int i=0;i<3;i++)
 {current_state_acc[i]=desired_state_acc[i]+ Kp*error_pos[i] +Kd*error_vel[i];}
vector<vector<double>> inverse_matrix;
inverse_matrix[0][0]=-1.1547344;
inverse_matrix[0][1]=1.333;
inverse_matrix[0][2]=0.3333;
inverse_matrix[1][0]=0;
inverse_matrix[1][1]=-0.6666;
inverse_matrix[1][2]=0.333;
inverse_matrix[2][0]=1.1547344;
inverse_matrix[2][1]=-0.6666;
inverse_matrix[2][2]=0.333;

for(int i=0;i<3;i++)
{
 f[i]=((m*current_state_acc[0])*inverse_matrix[i][0]+(m*current_state_acc[1])*inverse_matrix[i][1]+(Izz*current_state_acc[2])*inverse_matrix[i][2])/d;
}
omega_wheel.push_back(sqrt(f[0]/k));
omega_wheel.push_back(sqrt(f[1]/k));
omega_wheel.push_back(sqrt(f[2]/k));

}


int main(int argc, char **argv){

	ros::init(argc, argv, "pid_node");
	ros::NodeHandle n;

	n.getParam("~_ns", bot_name);

	String traj_topic = 

	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
	ros::Subscriber traj_pub = n.advertise<nav_msgs::Path>("trajectory", 50);

	ros::Rate r(50);

	while(n.ok()){

		ros::spinOnce();

		nav_msgs::Path traj;

		

	}


}